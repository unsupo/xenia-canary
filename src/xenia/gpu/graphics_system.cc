/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/gpu/graphics_system.h"

#include <chrono>

#include "xenia/base/byte_stream.h"
#include "xenia/base/clock.h"
#include "xenia/base/logging.h"
#include "xenia/base/math.h"
#include "xenia/base/profiling.h"
#include "xenia/base/threading.h"
#include "xenia/config.h"
#include "xenia/gpu/command_processor.h"
#include "xenia/gpu/gpu_flags.h"
#include "xenia/kernel/kernel_state.h"
#include "xenia/ui/graphics_provider.h"
#include "xenia/ui/window.h"
#include "xenia/ui/windowed_app_context.h"

DEFINE_uint32(custom_internal_display_resolution_x, 0,
              "Custom width. See internal_display_resolution. Range 1-1920.",
              "Video");
DEFINE_uint32(custom_internal_display_resolution_y, 0,
              "Custom height. See internal_display_resolution. Range 1-1080.\n",
              "Video");

DEFINE_bool(
    store_shaders, true,
    "Store shaders persistently and load them when loading games to avoid "
    "runtime spikes and freezes when playing the game not for the first time.",
    "GPU.Debug");

namespace xe {
namespace gpu {

// Nvidia Optimus/AMD PowerXpress support.
// These exports force the process to trigger the discrete GPU in multi-GPU
// systems.
// https://developer.download.nvidia.com/devzone/devcenter/gamegraphics/files/OptimusRenderingPolicies.pdf
// https://stackoverflow.com/questions/17458803/amd-equivalent-to-nvoptimusenablement
#if XE_PLATFORM_WIN32
extern "C" {
__declspec(dllexport) uint32_t NvOptimusEnablement = 0x00000001;
__declspec(dllexport) uint32_t AmdPowerXpressRequestHighPerformance = 1;
}  // extern "C"
#endif  // XE_PLATFORM_WIN32

GraphicsSystem::GraphicsSystem() : frame_limiter_worker_running_(false) {
  register_file_ = reinterpret_cast<RegisterFile*>(memory::AllocFixed(
      nullptr, sizeof(RegisterFile), memory::AllocationType::kReserveCommit,
      memory::PageAccess::kReadWrite));
}

GraphicsSystem::~GraphicsSystem() = default;

X_STATUS GraphicsSystem::Setup(cpu::Processor* processor,
                               kernel::KernelState* kernel_state,
                               ui::WindowedAppContext* app_context,
                               bool with_presentation) {
  memory_ = processor->memory();
  processor_ = processor;
  kernel_state_ = kernel_state;
  app_context_ = app_context;

  scaled_aspect_x_ = 16;
  scaled_aspect_y_ = 9;

  if (with_presentation && provider_) {
    // Safe if either the UI thread call or the presenter creation fails.
    if (app_context_) {
      app_context_->CallInUIThreadSynchronous([this]() {
        presenter_ = provider_->CreatePresenter(
            [this](bool is_responsible, bool statically_from_ui_thread) {
              OnHostGpuLossFromAnyThread(is_responsible);
            });
      });
    } else {
      // May be needed for offscreen use, such as capturing the guest output
      // image.
      presenter_ = provider_->CreatePresenter(
          [this](bool is_responsible, bool statically_from_ui_thread) {
            OnHostGpuLossFromAnyThread(is_responsible);
          });
    }
  }

  // Create command processor. This will spin up a thread to process all
  // incoming ringbuffer packets.
  command_processor_ = CreateCommandProcessor();
  if (!command_processor_->Initialize()) {
    XELOGE("Unable to initialize command processor");
    return X_STATUS_UNSUCCESSFUL;
  }

  // Let the processor know we want register access callbacks.
  memory_->AddVirtualMappedRange(
      0x7FC80000, 0xFFFF0000, 0x0000FFFF, this,
      reinterpret_cast<cpu::MMIOReadCallback>(ReadRegisterThunk),
      reinterpret_cast<cpu::MMIOWriteCallback>(WriteRegisterThunk));

  // Frame limiter thread.
  frame_limiter_worker_running_ = true;
  frame_limiter_worker_thread_ =
      kernel::object_ref<kernel::XHostThread>(new kernel::XHostThread(
          kernel_state_, 128 * 1024, 0,
          [this]() {
            uint64_t normalized_framerate_limit =
                std::max<uint64_t>(0, cvars::framerate_limit);

            // If VSYNC is enabled, but frames are not limited,
            // lock framerate at default value of 60
            if (normalized_framerate_limit == 0 && cvars::vsync) {
              normalized_framerate_limit = 60;
            }

            const double vsync_duration_d =
                cvars::vsync
                    ? std::max<double>(5.0,
                                       1000.0 / static_cast<double>(
                                                    normalized_framerate_limit))
                    : 1.0;
            uint64_t last_frame_time = Clock::QueryGuestTickCount();
    // Sleep for 90% of the vblank duration on Windows, spin for 10%
    // Linux uses full sleep duration due to scheduler quantum issues
#if XE_PLATFORM_WIN32
            constexpr double duration_scalar = 0.90;
#endif
#if XE_PLATFORM_LINUX
            constexpr double duration_scalar = 1.0;
#endif

            while (frame_limiter_worker_running_) {
              // If there is no title running then there is no need for guest
              // frame limiter thread.
              if (!kernel_state_->is_title_open()) {
                xe::threading::Sleep(std::chrono::milliseconds(100));
                continue;
              }

              register_file()->values[XE_GPU_REG_D1MODE_V_COUNTER] +=
                  GetResolution().second;

#if XE_PLATFORM_WIN32
              if (cvars::vsync) {
                const uint64_t current_time = Clock::QueryGuestTickCount();
                const uint64_t tick_freq = Clock::guest_tick_frequency();
                const uint64_t time_delta = current_time - last_frame_time;
                const double elapsed_d =
                    static_cast<double>(time_delta) /
                    (static_cast<double>(tick_freq) / 1000.0);
                if (elapsed_d >= vsync_duration_d) {
                  last_frame_time = current_time;

                  MarkVblank();
                  const uint64_t estimated_nanoseconds = static_cast<uint64_t>(
                      (vsync_duration_d * 1000000.0) *
                      duration_scalar);  // 1000 microseconds = 1 ms

                  threading::NanoSleep(estimated_nanoseconds);
                }
              }

              if (!cvars::vsync) {
                MarkVblank();
                if (normalized_framerate_limit > 0) {
                  // framerate_limit is over 0, vsync disabled
                  //  - No VSYNC + limited frames defined by user
                  uint64_t framerate_limited_sleep_time =
                      1000000000 / normalized_framerate_limit;
                  xe::threading::NanoSleep(framerate_limited_sleep_time);
                } else {
                  // framerate_limit is 0, vsync disabled
                  //  - No VSYNC + unlimited frames
                  xe::threading::Sleep(std::chrono::milliseconds(1));
                }
              }
#endif
#if XE_PLATFORM_LINUX || XE_PLATFORM_MAC
              // POSIX/macOS: Use simplified timing logic to avoid oversleeping
              MarkVblank();

              if (cvars::vsync || normalized_framerate_limit > 0) {
                uint64_t sleep_duration_ns =
                    static_cast<uint64_t>(vsync_duration_d * 1000000.0);
                if (!cvars::vsync && normalized_framerate_limit > 0) {
                  sleep_duration_ns = 1000000000 / normalized_framerate_limit;
                }
                threading::NanoSleep(sleep_duration_ns);
              } else {
                xe::threading::Sleep(std::chrono::milliseconds(1));
              }
#endif
            }
            return 0;
          },
          kernel_state->GetIdleProcess()));
  // As we run vblank interrupts the debugger must be able to suspend us.
  frame_limiter_worker_thread_->set_can_debugger_suspend(true);
  frame_limiter_worker_thread_->set_name("GPU Frame limiter");
  frame_limiter_worker_thread_->Create();
#if XE_PLATFORM_MAC
  // macOS maps ThreadPriority::kLowest to nice +19, which the scheduler will
  // starve almost completely whenever another thread is busy-spinning at
  // 100% - and guest GPU-fence waits are exactly that. This thread delivers
  // the vblank interrupts the guest's frame pacing depends on, so starving
  // it livelocks the title (the spinning waiter monopolises the CPU and the
  // signaller never runs). Keep it at normal priority here.
  frame_limiter_worker_thread_->thread()->set_priority(
      threading::ThreadPriority::kNormal);
#else
  frame_limiter_worker_thread_->thread()->set_priority(
      threading::ThreadPriority::kLowest);
#endif
  if (cvars::trace_gpu_stream) {
    BeginTracing();
  }

  return X_STATUS_SUCCESS;
}

bool GraphicsSystem::InitializeOffscreenPresenter() {
  if (presenter_) {
    return true;
  }
  if (!provider_) {
    return false;
  }
  presenter_ = provider_->CreatePresenter(
      [this](bool is_responsible, bool statically_from_ui_thread) {
        OnHostGpuLossFromAnyThread(is_responsible);
      });
  return presenter_ != nullptr;
}

void GraphicsSystem::Shutdown() {
  if (command_processor_) {
    EndTracing();
    command_processor_->Shutdown();
    command_processor_.reset();
  }

  if (frame_limiter_worker_thread_) {
    frame_limiter_worker_running_ = false;
    frame_limiter_worker_thread_->Wait(0, 0, 0, nullptr);
    frame_limiter_worker_thread_.reset();
  }

  if (presenter_) {
    if (app_context_) {
      app_context_->CallInUIThreadSynchronous([this]() { presenter_.reset(); });
    }
    // If there's no app context (thus the presenter is owned by the thread that
    // initialized the GraphicsSystem) or can't be queueing UI thread calls
    // anymore, shutdown anyway.
    presenter_.reset();
  }

  provider_.reset();
}

void GraphicsSystem::OnHostGpuLossFromAnyThread(
    [[maybe_unused]] bool is_responsible) {
  // TODO(Triang3l): Somehow gain exclusive ownership of the Provider (may be
  // used by the command processor, the presenter, and possibly anything else,
  // it's considered free-threaded, except for lifetime management which will be
  // involved in this case) and reset it so a new host GPU API device is
  // created. Then ask the command processor to reset itself in its thread, and
  // ask the UI thread to reset the Presenter (the UI thread manages its
  // lifetime - but if there's no WindowedAppContext, either don't reset it as
  // in this case there's no user who needs uninterrupted gameplay, or somehow
  // protect it with a mutex so any thread can be considered a UI thread and
  // reset).
  if (host_gpu_loss_reported_.test_and_set(std::memory_order_relaxed)) {
    return;
  }

  config::SaveConfig();

  xe::FatalError("Graphics device lost (probably due to an internal error)");
}

uint32_t GraphicsSystem::ReadRegisterThunk(void* ppc_context,
                                           GraphicsSystem* gs, uint32_t addr) {
  return gs->ReadRegister(addr);
}

void GraphicsSystem::WriteRegisterThunk(void* ppc_context, GraphicsSystem* gs,
                                        uint32_t addr, uint32_t value) {
  gs->WriteRegister(addr, value);
}

uint32_t GraphicsSystem::ReadRegister(uint32_t addr) {
  uint32_t r = (addr & 0xFFFF) / 4;

  switch (r) {
    case 0x0F00:  // RB_EDRAM_TIMING
      return 0x08100748;
    case 0x0F01:  // RB_BC_CONTROL
      return 0x0000200E;
    case 0x1951:  // interrupt status
      return 1;   // vblank
    case 0x1961:  // AVIVO_D1MODE_VIEWPORT_SIZE
                  // Screen res - 1280x720
                  // maximum [width(0x0FFF), height(0x0FFF)]
      return 0x050002D0;
    default:
      if (!register_file()->IsValidRegister(r)) {
        XELOGE("GPU: Read from unknown register ({:04X})", r);
      }
  }

  assert_true(r < RegisterFile::kRegisterCount);
  return register_file()->values[r];
}

void GraphicsSystem::WriteRegister(uint32_t addr, uint32_t value) {
  uint32_t r = (addr & 0xFFFF) / 4;

  switch (r) {
    case 0x01C5:  // CP_RB_WPTR
      command_processor_->UpdateWritePointer(value);
      break;
    case 0x1844:  // AVIVO_D1GRPH_PRIMARY_SURFACE_ADDRESS
      break;
    default:
      XELOGW("Unknown GPU register {:04X} write: {:08X}", r, value);
      break;
  }

  assert_true(r < RegisterFile::kRegisterCount);
  this->register_file()->values[r] = value;
}

void GraphicsSystem::InitializeRingBuffer(uint32_t ptr, uint32_t size_log2) {
  command_processor_->InitializeRingBuffer(ptr, size_log2);
}

void GraphicsSystem::EnableReadPointerWriteBack(uint32_t ptr,
                                                uint32_t block_size_log2) {
  command_processor_->EnableReadPointerWriteBack(ptr, block_size_log2);
}

void GraphicsSystem::SetInterruptCallback(uint32_t callback,
                                          uint32_t user_data) {
  interrupt_callback_ = callback;
  interrupt_callback_data_ = user_data;
  // The interrupt user_data is the guest's D3D "swap sync" object. Its
  // fields drive the frame-pacing fence: +0x2a90 -> pointer to the value
  // the guest polls, +0x2a9c -> the value it wants that pointer to reach.
  swap_sync_object_ = user_data;
}

void GraphicsSystem::DispatchInterruptCallback(uint32_t source, uint32_t cpu) {
  kernel_state()->EmulateCPInterruptDPC(interrupt_callback_,
                                        interrupt_callback_data_, source, cpu);
}

void GraphicsSystem::SetGpuIdentifierAddress(uint32_t guest_address) {
  gpu_identifier_address_.store(guest_address, std::memory_order_relaxed);
  PublishGpuIdentifier();
}

void GraphicsSystem::SetGpuIdentifierValue(uint32_t value) {
  // Monotonic - never let it go backwards (the CP can process an older
  // write_ptr snapshot after a newer one on a wraparound edge).
  uint32_t prev = gpu_identifier_value_.load(std::memory_order_relaxed);
  while (int32_t(value - prev) > 0 &&
         !gpu_identifier_value_.compare_exchange_weak(
             prev, value, std::memory_order_relaxed)) {
  }
  PublishGpuIdentifier();
}

// Read/write a guest u32 (big-endian) applying the same +0x1000 fixup the
// A64 JIT does for >= 0xE0000000 addresses on 16 KB-page hosts (see
// a64_seq_util.h ComputeMemoryAddress) so we hit the page the guest reads.
static inline uint8_t* GuestU32Host(uint8_t* base, uint32_t guest_addr) {
  if (guest_addr >= 0xE0000000 && xe::memory::allocation_granularity() > 0x1000) {
    guest_addr += 0x1000;
  }
  return base + guest_addr;
}

void GraphicsSystem::PublishGpuIdentifier() {
  uint8_t* base = memory_->virtual_membase();
  // Resolve the frame-pacing fence straight from the guest's own sync object:
  // write the value it's waiting FOR (its target) into the location it polls.
  // We have no real GPU timeline, so the pacing fence is a no-op - the guest
  // stays gated by actually getting scheduled and by the CP draining the ring.
  uint32_t addr = gpu_identifier_address_.load(std::memory_order_relaxed);
  if (!addr) {
    return;
  }
  // The guest passes an 0xE0000000-window (or physical) address; both resolve
  // through TranslateVirtual for the guest's own view. Direct3D spins reading
  // this expecting the GPU's completion counter, which for us is the
  // swap/interrupt counter the command processor maintains. The guest polls a
  // couple of words around the address it registered, so cover the small
  // record.
  // value is the count of guest submissions the CP has fully drained. The
  // guest's fence wants completed >= submitted - small_allowance, so this
  // exact count resolves it. Do NOT add a lead - overshooting the guest's
  // submitted count wraps its unsigned compare and spins just the same.
  uint32_t value = gpu_identifier_value_.load(std::memory_order_relaxed);
  // Match the JIT's guest->host address computation exactly (see
  // a64_seq_util.h ComputeMemoryAddress): on hosts whose allocation
  // granularity exceeds 4 KB (macOS ARM64), guest addresses at/above
  // 0xE0000000 are shifted +0x1000. TranslateVirtual's heap-offset path does
  // NOT reproduce this for the 0xE0000000 window, so the guest's read of
  // this address and a naive write would land on different host pages.
  // Write only the identifier word the guest registered. Do NOT spray
  // neighbouring words - the guest keeps other GPU-status fields (tagged
  // sequences) right next to it.
  *reinterpret_cast<uint32_t*>(GuestU32Host(base, addr)) =
      __builtin_bswap32(value);

  // Direct3D's frame-pacing fence.
  //
  // Fable II (and other D3D titles) register a graphics-interrupt sync object
  // via VdSetGraphicsInterruptCallback whose user_data we stashed in
  // swap_sync_object_. Its layout:
  //   +0x2a90  poll pointer  (an 0xE0000000-window guest address)
  //   +0x2a9c  target        (the fence value the title is spinning FOR)
  // The title's spin loop (guest 0x82242628 / 0x82B9D380) exits when the word
  // at *poll_ptr lands in [target - maxLag, target] (observed maxLag: 2 for the
  // outer driver, 4 for the inner dispatcher). The word is normally advanced by
  // the title's own EVENT_WRITE_SHD packets - but once the CP has drained the
  // ring the title won't submit the work that would post the last increment(s)
  // until the fence passes: a circular wait with no async GPU to break it.
  //
  // Snap the poll word up to the target once it has fallen clearly behind
  // (lag > 2, i.e. past the tightest observed maxLag). We have already drained
  // every packet the CP was handed, so from the guest's point of view the GPU
  // *is* caught up; the residual lag is the title reserving fence slots ahead
  // of the EVENT_WRITE_SHD packets it will only emit once this fence clears.
  // During normal streaming EVENT_WRITE_SHD keeps the word within a couple of
  // the target, so this never fights the title's own writes. Writing exactly
  // target (never past it) keeps the guest's unsigned target-relative compare
  // from wrapping.
  uint32_t sync = swap_sync_object_.load(std::memory_order_relaxed);
  if (sync) {
    auto sync_u32 = [&](uint32_t off) {
      return __builtin_bswap32(
          *reinterpret_cast<uint32_t*>(base + sync + off));
    };
    uint32_t poll_ptr = sync_u32(0x2a90);
    uint32_t target = sync_u32(0x2a9c);
    if (poll_ptr >= 0xE0000000) {
      uint8_t* host = GuestU32Host(base, poll_ptr);
      uint32_t live = __builtin_bswap32(*reinterpret_cast<uint32_t*>(host));
      if (static_cast<int32_t>(target - live) > 2) {
        *reinterpret_cast<uint32_t*>(host) = __builtin_bswap32(target);
      }

      // poll_ptr[+4] is the second frame-pacing fence: the guest's D3D
      // command-buffer allocator (guest 0x82206538 / 0x822065F0) spins here
      // waiting for the "GPU consumed pointer" - normally advanced by the
      // title's own EVENT_WRITE_SHD(addr=...+4) packets - to reach the region
      // it wants to (re)allocate. Its low 2 bits are a wrap-generation tag;
      // the allocator's exit test is purely `(gen - word) & 3 == 0`, where
      // gen = *(sync + 0x3a48). Once the CP has drained the ring there is no
      // pending GPU work, so from the guest's view the GPU *is* in the
      // current generation - publish that by matching the tag. Gate on an
      // actually-drained ring so we never signal "consumed" while the CP is
      // still reading command memory the allocator is about to reuse.
      if (command_processor_ && command_processor_->is_ring_idle()) {
        uint8_t* host4 = GuestU32Host(base, poll_ptr + 4);
        uint32_t word4 = __builtin_bswap32(*reinterpret_cast<uint32_t*>(host4));
        uint32_t gen_tag = sync_u32(0x3a48) & 0x3;
        if ((word4 & 0x3) != gen_tag) {
          *reinterpret_cast<uint32_t*>(host4) =
              __builtin_bswap32((word4 & ~0x3u) | gen_tag);
        }
      }
    }
  }
}

void GraphicsSystem::MarkVblank() {
  SCOPE_profile_cpu_f("gpu");

  // Increment vblank counter (so the game sees us making progress).
  command_processor_->increment_counter();
  PublishGpuIdentifier();

  // TODO(benvanik): we shouldn't need to do the dispatch here, but there's
  //     something wrong and the CP will block waiting for code that
  //     needs to be run in the interrupt.
  DispatchInterruptCallback(0, 2);
}

void GraphicsSystem::ClearCaches() {
  command_processor_->CallInThread(
      [&]() { command_processor_->ClearCaches(); });
}

void GraphicsSystem::InitializeShaderStorage(
    const std::filesystem::path& cache_root, uint32_t title_id, bool blocking,
    std::function<void()> completion_callback) {
  if (!cvars::store_shaders) {
    if (completion_callback) {
      completion_callback();
    }
    return;
  }
  if (blocking) {
    if (command_processor_->is_paused()) {
      // Safe to run on any thread while the command processor is paused, no
      // race condition.
      command_processor_->InitializeShaderStorage(
          cache_root, title_id, true, std::move(completion_callback));
    } else {
      xe::threading::Fence fence;
      command_processor_->CallInThread(
          [this, cache_root, title_id, &fence,
           completion_callback = std::move(completion_callback)]() mutable {
            command_processor_->InitializeShaderStorage(
                cache_root, title_id, true, std::move(completion_callback));
            fence.Signal();
          });
      fence.Wait();
    }
  } else {
    command_processor_->CallInThread(
        [this, cache_root, title_id,
         completion_callback = std::move(completion_callback)]() mutable {
          command_processor_->InitializeShaderStorage(
              cache_root, title_id, false, std::move(completion_callback));
        });
  }
}

void GraphicsSystem::RequestFrameTrace() {
  command_processor_->RequestFrameTrace(cvars::trace_gpu_prefix);
}

void GraphicsSystem::BeginTracing() {
  command_processor_->BeginTracing(cvars::trace_gpu_prefix);
}

void GraphicsSystem::EndTracing() { command_processor_->EndTracing(); }

void GraphicsSystem::Pause() {
  paused_ = true;

  command_processor_->Pause();
}

void GraphicsSystem::Resume() {
  paused_ = false;

  command_processor_->Resume();
}

bool GraphicsSystem::Save(ByteStream* stream) {
  stream->Write<uint32_t>(interrupt_callback_);
  stream->Write<uint32_t>(interrupt_callback_data_);

  return command_processor_->Save(stream);
}

bool GraphicsSystem::Restore(ByteStream* stream) {
  interrupt_callback_ = stream->Read<uint32_t>();
  interrupt_callback_data_ = stream->Read<uint32_t>();

  return command_processor_->Restore(stream);
}

std::pair<uint32_t, uint32_t> GraphicsSystem::GetResolution() const {
  if (!kernel_state_) {
    return {1280, 720};
  }

  if (cvars::custom_internal_display_resolution_x != 0 &&
      cvars::custom_internal_display_resolution_y != 0) {
    return {cvars::custom_internal_display_resolution_x,
            cvars::custom_internal_display_resolution_y};
  }

  const auto resolution =
      kernel::Resolution(kernel_state()->xconfig()->ReadSetting<uint32_t>(
          kernel::XCONFIG_USER_CATEGORY,
          kernel::XCONFIG_USER_AV_COMPOSITE_SCREENSZ));

  return {resolution.width_, resolution.height_};
}

}  // namespace gpu
}  // namespace xe
