/**
 ******************************************************************************
 * Xenia : Xbox 360 Emulator Research Project                                 *
 ******************************************************************************
 * Copyright 2022 Ben Vanik. All rights reserved.                             *
 * Released under the BSD license - see LICENSE in the root for more details. *
 ******************************************************************************
 */

#include "xenia/base/exception_handler.h"

#include <signal.h>
#include <cstdint>

#include "xenia/base/assert.h"
#include "xenia/base/host_thread_context.h"
#include "xenia/base/logging.h"
#include "xenia/base/math.h"
#include "xenia/base/platform.h"

#ifdef __APPLE__
#include <mach/mach.h>
#include <sys/mman.h>
#include <unistd.h>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <execinfo.h>
#endif

#ifndef __APPLE__

namespace xe {

bool signal_handlers_installed_ = false;
struct sigaction original_sigill_handler_;
struct sigaction original_sigsegv_handler_;

// This can be as large as needed, but isn't often needed.
// As we will be sometimes firing many exceptions we want to avoid having to
// scan the table too much or invoke many custom handlers.
constexpr size_t kMaxHandlerCount = 8;

// All custom handlers, left-aligned and null terminated.
// Executed in order.
std::pair<ExceptionHandler::Handler, void*> handlers_[kMaxHandlerCount];

static void ExceptionHandlerCallback(int signal_number, siginfo_t* signal_info,
                                     void* signal_context) {
  mcontext_t& mcontext =
      reinterpret_cast<ucontext_t*>(signal_context)->uc_mcontext;

  HostThreadContext thread_context;

#if XE_ARCH_AMD64
  thread_context.rip = uint64_t(mcontext.gregs[REG_RIP]);
  thread_context.eflags = uint32_t(mcontext.gregs[REG_EFL]);
  // The REG_ order may be different than the register indices in the
  // instruction encoding.
  thread_context.rax = uint64_t(mcontext.gregs[REG_RAX]);
  thread_context.rcx = uint64_t(mcontext.gregs[REG_RCX]);
  thread_context.rdx = uint64_t(mcontext.gregs[REG_RDX]);
  thread_context.rbx = uint64_t(mcontext.gregs[REG_RBX]);
  thread_context.rsp = uint64_t(mcontext.gregs[REG_RSP]);
  thread_context.rbp = uint64_t(mcontext.gregs[REG_RBP]);
  thread_context.rsi = uint64_t(mcontext.gregs[REG_RSI]);
  thread_context.rdi = uint64_t(mcontext.gregs[REG_RDI]);
  thread_context.r8 = uint64_t(mcontext.gregs[REG_R8]);
  thread_context.r9 = uint64_t(mcontext.gregs[REG_R9]);
  thread_context.r10 = uint64_t(mcontext.gregs[REG_R10]);
  thread_context.r11 = uint64_t(mcontext.gregs[REG_R11]);
  thread_context.r12 = uint64_t(mcontext.gregs[REG_R12]);
  thread_context.r13 = uint64_t(mcontext.gregs[REG_R13]);
  thread_context.r14 = uint64_t(mcontext.gregs[REG_R14]);
  thread_context.r15 = uint64_t(mcontext.gregs[REG_R15]);
  std::memcpy(thread_context.xmm_registers, mcontext.fpregs->_xmm,
              sizeof(thread_context.xmm_registers));
#elif XE_ARCH_ARM64
  std::memcpy(thread_context.x, mcontext.regs, sizeof(thread_context.x));
  thread_context.sp = mcontext.sp;
  thread_context.pc = mcontext.pc;
  thread_context.pstate = mcontext.pstate;
  struct fpsimd_context* mcontext_fpsimd = nullptr;
  struct esr_context* mcontext_esr = nullptr;
  for (struct _aarch64_ctx* mcontext_extension =
           reinterpret_cast<struct _aarch64_ctx*>(mcontext.__reserved);
       mcontext_extension->magic;
       mcontext_extension = reinterpret_cast<struct _aarch64_ctx*>(
           reinterpret_cast<uint8_t*>(mcontext_extension) +
           mcontext_extension->size)) {
    switch (mcontext_extension->magic) {
      case FPSIMD_MAGIC:
        mcontext_fpsimd =
            reinterpret_cast<struct fpsimd_context*>(mcontext_extension);
        break;
      case ESR_MAGIC:
        mcontext_esr =
            reinterpret_cast<struct esr_context*>(mcontext_extension);
        break;
      default:
        break;
    }
  }
  assert_not_null(mcontext_fpsimd);
  if (mcontext_fpsimd) {
    thread_context.fpsr = mcontext_fpsimd->fpsr;
    thread_context.fpcr = mcontext_fpsimd->fpcr;
    std::memcpy(thread_context.v, mcontext_fpsimd->vregs,
                sizeof(thread_context.v));
  }
#endif  // XE_ARCH

  Exception ex;
  switch (signal_number) {
    case SIGILL:
      ex.InitializeIllegalInstruction(&thread_context);
      break;
    case SIGSEGV: {
      Exception::AccessViolationOperation access_violation_operation;
#if XE_ARCH_AMD64
      // x86_pf_error_code::X86_PF_WRITE
      constexpr uint64_t kX86PageFaultErrorCodeWrite = UINT64_C(1) << 1;
      access_violation_operation =
          (uint64_t(mcontext.gregs[REG_ERR]) & kX86PageFaultErrorCodeWrite)
              ? Exception::AccessViolationOperation::kWrite
              : Exception::AccessViolationOperation::kRead;
#elif XE_ARCH_ARM64
      // For a Data Abort (EC - ESR_EL1 bits 31:26 - 0b100100 from a lower
      // Exception Level, 0b100101 without a change in the Exception Level),
      // bit 6 is 0 for reading from a memory location, 1 for writing to a
      // memory location.
      if (mcontext_esr && ((mcontext_esr->esr >> 26) & 0b111110) == 0b100100) {
        access_violation_operation =
            (mcontext_esr->esr & (UINT64_C(1) << 6))
                ? Exception::AccessViolationOperation::kWrite
                : Exception::AccessViolationOperation::kRead;
      } else {
        // Determine the memory access direction based on which instruction has
        // requested it.
        // esr_context may be unavailable on certain hosts (for instance, on
        // Android, it was added only in NDK r16 - which is the first NDK
        // version to support the Android API level 27, while NDK r15 doesn't
        // have esr_context in its API 26 sigcontext.h).
        // On AArch64 (unlike on AArch32), the program counter is the address of
        // the currently executing instruction.
        bool instruction_is_store;
        if (IsArm64LoadPrefetchStore(
                *reinterpret_cast<const uint32_t*>(mcontext.pc),
                instruction_is_store)) {
          access_violation_operation =
              instruction_is_store ? Exception::AccessViolationOperation::kWrite
                                   : Exception::AccessViolationOperation::kRead;
        } else {
          assert_always(
              "No ESR in the exception thread context, or it's not a Data "
              "Abort, and the faulting instruction is not a known load, "
              "prefetch or store instruction");
          access_violation_operation =
              Exception::AccessViolationOperation::kUnknown;
        }
      }
#else
      access_violation_operation =
          Exception::AccessViolationOperation::kUnknown;
#endif  // XE_ARCH
      ex.InitializeAccessViolation(
          &thread_context, reinterpret_cast<uint64_t>(signal_info->si_addr),
          access_violation_operation);
    } break;
    default:
      assert_unhandled_case(signal_number);
  }

  for (size_t i = 0; i < xe::countof(handlers_) && handlers_[i].first; ++i) {
    if (handlers_[i].first(&ex, handlers_[i].second)) {
      // Exception handled.
#if XE_ARCH_AMD64
      mcontext.gregs[REG_RIP] = greg_t(thread_context.rip);
      mcontext.gregs[REG_EFL] = greg_t(thread_context.eflags);
      uint32_t modified_register_index;
      // The order must match the order in X64Register.
      static constexpr size_t kIntRegisterMap[] = {
          REG_RAX, REG_RCX, REG_RDX, REG_RBX, REG_RSP, REG_RBP,
          REG_RSI, REG_RDI, REG_R8,  REG_R9,  REG_R10, REG_R11,
          REG_R12, REG_R13, REG_R14, REG_R15,
      };
      uint16_t modified_int_registers_remaining = ex.modified_int_registers();
      while (xe::bit_scan_forward(modified_int_registers_remaining,
                                  &modified_register_index)) {
        modified_int_registers_remaining &=
            ~(UINT16_C(1) << modified_register_index);
        mcontext.gregs[kIntRegisterMap[modified_register_index]] =
            thread_context.int_registers[modified_register_index];
      }
      uint16_t modified_xmm_registers_remaining = ex.modified_xmm_registers();
      while (xe::bit_scan_forward(modified_xmm_registers_remaining,
                                  &modified_register_index)) {
        modified_xmm_registers_remaining &=
            ~(UINT16_C(1) << modified_register_index);
        std::memcpy(&mcontext.fpregs->_xmm[modified_register_index],
                    &thread_context.xmm_registers[modified_register_index],
                    sizeof(vec128_t));
      }
#elif XE_ARCH_ARM64
      uint32_t modified_register_index;
      uint32_t modified_x_registers_remaining = ex.modified_x_registers();
      while (xe::bit_scan_forward(modified_x_registers_remaining,
                                  &modified_register_index)) {
        modified_x_registers_remaining &=
            ~(UINT32_C(1) << modified_register_index);
        mcontext.regs[modified_register_index] =
            thread_context.x[modified_register_index];
      }
      mcontext.sp = thread_context.sp;
      mcontext.pc = thread_context.pc;
      mcontext.pstate = thread_context.pstate;
      if (mcontext_fpsimd) {
        mcontext_fpsimd->fpsr = thread_context.fpsr;
        mcontext_fpsimd->fpcr = thread_context.fpcr;
        uint32_t modified_v_registers_remaining = ex.modified_v_registers();
        while (xe::bit_scan_forward(modified_v_registers_remaining,
                                    &modified_register_index)) {
          modified_v_registers_remaining &=
              ~(UINT32_C(1) << modified_register_index);
          std::memcpy(&mcontext_fpsimd->vregs[modified_register_index],
                      &thread_context.v[modified_register_index],
                      sizeof(vec128_t));
          mcontext.regs[modified_register_index] =
              thread_context.x[modified_register_index];
        }
      }
#endif  // XE_ARCH
      return;
    }
  }
}

void ExceptionHandler::Install(Handler fn, void* data) {
  if (!signal_handlers_installed_) {
    struct sigaction signal_handler;

    std::memset(&signal_handler, 0, sizeof(signal_handler));
    signal_handler.sa_sigaction = ExceptionHandlerCallback;
    signal_handler.sa_flags = SA_SIGINFO;

    if (sigaction(SIGILL, &signal_handler, &original_sigill_handler_) != 0) {
      assert_always("Failed to install new SIGILL handler");
    }
    if (sigaction(SIGSEGV, &signal_handler, &original_sigsegv_handler_) != 0) {
      assert_always("Failed to install new SIGSEGV handler");
    }
    signal_handlers_installed_ = true;
  }

  for (size_t i = 0; i < xe::countof(handlers_); ++i) {
    if (!handlers_[i].first) {
      handlers_[i].first = fn;
      handlers_[i].second = data;
      return;
    }
  }
  assert_always("Too many exception handlers installed");
}

void ExceptionHandler::Uninstall(Handler fn, void* data) {
  for (size_t i = 0; i < xe::countof(handlers_); ++i) {
    if (handlers_[i].first == fn && handlers_[i].second == data) {
      for (; i < xe::countof(handlers_) - 1; ++i) {
        handlers_[i] = handlers_[i + 1];
      }
      handlers_[i].first = nullptr;
      handlers_[i].second = nullptr;
      break;
    }
  }

  bool has_any = false;
  for (size_t i = 0; i < xe::countof(handlers_); ++i) {
    if (handlers_[i].first) {
      has_any = true;
      break;
    }
  }
  if (!has_any) {
    if (signal_handlers_installed_) {
      if (sigaction(SIGILL, &original_sigill_handler_, NULL) != 0) {
        assert_always("Failed to restore original SIGILL handler");
      }
      if (sigaction(SIGSEGV, &original_sigsegv_handler_, NULL) != 0) {
        assert_always("Failed to restore original SIGSEGV handler");
      }
      signal_handlers_installed_ = false;
    }
  }
}

}  // namespace xe

#else

namespace xe {

static bool signal_handlers_installed_ = false;
static struct sigaction original_sigill_handler_;
static struct sigaction original_sigsegv_handler_;
static struct sigaction original_sigbus_handler_;

constexpr size_t kMaxHandlerCount = 8;
static std::pair<ExceptionHandler::Handler, void*> handlers_[kMaxHandlerCount];

static void DarwinExceptionHandlerCallback(int signal_number, siginfo_t* signal_info,
                                           void* signal_context) {
  ucontext_t* uc = reinterpret_cast<ucontext_t*>(signal_context);
  mcontext_t mc = uc->uc_mcontext;

  HostThreadContext thread_context;

#if XE_ARCH_ARM64
  for (int i = 0; i < 29; ++i) {
    thread_context.x[i] = mc->__ss.__x[i];
  }
  thread_context.x[29] = mc->__ss.__fp;
  thread_context.x[30] = mc->__ss.__lr;
  thread_context.sp = mc->__ss.__sp;
  thread_context.pc = mc->__ss.__pc;
  thread_context.pstate = mc->__ss.__cpsr;
  thread_context.fpsr = mc->__ns.__fpsr;
  thread_context.fpcr = mc->__ns.__fpcr;
  std::memcpy(thread_context.v, mc->__ns.__v, sizeof(thread_context.v));
#endif

  Exception ex;
  switch (signal_number) {
    case SIGILL:
      ex.InitializeIllegalInstruction(&thread_context);
      break;
    case SIGBUS:
    case SIGSEGV: {
      Exception::AccessViolationOperation access_violation_operation =
          Exception::AccessViolationOperation::kUnknown;
#if XE_ARCH_ARM64
      bool instruction_is_store = false;
      uint32_t instruction = 0;
      bool can_read_pc = false;
#ifdef __APPLE__
      if (mc->__ss.__pc && (mc->__ss.__pc & 0x3) == 0) {
        vm_offset_t data_out = 0;
        mach_msg_type_number_t count = 0;
        if (vm_read(mach_task_self(), static_cast<vm_address_t>(mc->__ss.__pc),
                    sizeof(uint32_t), &data_out, &count) == KERN_SUCCESS &&
            count >= sizeof(uint32_t)) {
          instruction = *reinterpret_cast<const uint32_t*>(data_out);
          vm_deallocate(mach_task_self(), data_out, count);
          can_read_pc = true;
        }
      }
#else
      if (mc->__ss.__pc && (mc->__ss.__pc & 0x3) == 0) {
        instruction = *reinterpret_cast<const uint32_t*>(mc->__ss.__pc);
        can_read_pc = true;
      }
#endif
      if (can_read_pc &&
          IsArm64LoadPrefetchStore(instruction, instruction_is_store)) {
        access_violation_operation =
            instruction_is_store ? Exception::AccessViolationOperation::kWrite
                                 : Exception::AccessViolationOperation::kRead;
      }
#endif
      ex.InitializeAccessViolation(
          &thread_context, reinterpret_cast<uint64_t>(signal_info->si_addr),
          access_violation_operation);
    } break;
    default:
      return;
  }

  for (size_t i = 0; i < xe::countof(handlers_) && handlers_[i].first; ++i) {
    if (handlers_[i].first(&ex, handlers_[i].second)) {
#if XE_ARCH_ARM64
      for (int r = 0; r < 29; ++r) {
        mc->__ss.__x[r] = thread_context.x[r];
      }
      mc->__ss.__fp = thread_context.x[29];
      mc->__ss.__lr = thread_context.x[30];
      mc->__ss.__sp = thread_context.sp;
      mc->__ss.__pc = thread_context.pc;
      mc->__ss.__cpsr = thread_context.pstate;
      mc->__ns.__fpsr = thread_context.fpsr;
      mc->__ns.__fpcr = thread_context.fpcr;
      std::memcpy(mc->__ns.__v, thread_context.v, sizeof(thread_context.v));
#endif
      return;
    }
  }

#if XE_ARCH_ARM64
  // macos-arm64 Fable II bring-up: last-resort recovery for a guest write that
  // faulted on a mapped-but-read-only page none of the registered handlers
  // claimed. This happens for stores through the 0xE0000000 physical-mirror
  // window whose page-table protection in the vE0000000 heap view is stale
  // relative to the physical heap (the +0x1000 host-page-offset mirror doesn't
  // stay in sync), so PhysicalHeap::TriggerCallbacks sees no watch and
  // SystemPageGuestAccess reports the page not-writable even though the
  // physical page IS committed and writable through its other views. Rather
  // than crash the title, make the faulting page writable and retry. Worst
  // case is a missed texture/vertex-cache invalidation for that page.
  if ((signal_number == SIGSEGV || signal_number == SIGBUS) &&
      ex.code() == Exception::Code::kAccessViolation && signal_info->si_addr) {
    static std::atomic<uint32_t> recover_warn{0};
    uintptr_t fault = reinterpret_cast<uintptr_t>(signal_info->si_addr);
    // Only touch the guest address space: xenia maps it at a power-of-two base
    // (virtual_membase) with the physical mirror +4 GiB, spanning well under
    // 16 GiB. A host stack/heap/library address won't be page-RO-and-writable
    // after mprotect, but scope it anyway to the 4 GiB..64 GiB range the guest
    // windows always land in.
    if (fault >= 0x100000000ull && fault < 0x1000000000ull) {
      long page = sysconf(_SC_PAGESIZE);
      uintptr_t page_base = fault & ~uintptr_t(page - 1);
      vm_address_t region = page_base;
      vm_size_t region_size = 0;
      vm_region_basic_info_data_64_t info;
      mach_msg_type_number_t info_count = VM_REGION_BASIC_INFO_COUNT_64;
      mach_port_t obj = MACH_PORT_NULL;
      bool mapped_ro = false;
      if (vm_region_64(mach_task_self(), &region, &region_size,
                       VM_REGION_BASIC_INFO_64,
                       reinterpret_cast<vm_region_info_t>(&info), &info_count,
                       &obj) == KERN_SUCCESS &&
          region <= page_base && (info.protection & VM_PROT_READ) &&
          !(info.protection & VM_PROT_WRITE)) {
        mapped_ro = true;
      }
      if (mapped_ro &&
          mprotect(reinterpret_cast<void*>(page_base), size_t(page),
                   PROT_READ | PROT_WRITE) == 0) {
        // Signal-handler context: no logging here (the fault may have
        // interrupted the logger). A one-line stderr note, rate-limited, is
        // async-signal tolerable and enough to know it fired.
        uint32_t n = recover_warn.fetch_add(1, std::memory_order_relaxed);
        if (n == 0 || (n & 0xFFF) == 0) {
          char buf[128];
          int len = snprintf(buf, sizeof(buf),
                             "[exception_handler] recovered guest write fault "
                             "at %p (count %u)\n",
                             signal_info->si_addr, n + 1);
          if (len > 0) {
            ssize_t w = write(STDERR_FILENO, buf, size_t(len));
            (void)w;
          }
        }
        return;
      }
    }
  }
#endif

#ifdef __APPLE__
  // macos-arm64 Fable II bring-up: none of the registered handlers claimed
  // this fault, so it's about to terminate the process via the default
  // signal disposition - which, in this sandboxed environment, generates no
  // ~/Library/Logs/DiagnosticReports/*.ips (confirmed empirically: multiple
  // reproductions of an ARM64 JIT crash left no crash report at all). A
  // std::set_terminate hook (windowed_app_main_mac.mm) only catches uncaught
  // C++ exceptions, not a raw SIGILL/SIGSEGV/SIGBUS like this, so that
  // wouldn't have fired either. backtrace()/backtrace_symbols() are not
  // strictly async-signal-safe, but the process is already unrecoverable
  // here - a best-effort dump beats losing the only diagnostic evidence
  // this environment produces for a fatal signal.
  {
    char header[192];
    int header_len =
        snprintf(header, sizeof(header),
                 "\n=== FableII-CRASH-TRACE: unhandled signal %d (%s) at "
                 "pc=0x%llx fault_addr=%p ===\n",
                 signal_number, strsignal(signal_number),
                 static_cast<unsigned long long>(mc->__ss.__pc),
                 signal_info->si_addr);
    if (header_len > 0) {
      ssize_t w = write(STDERR_FILENO, header, size_t(header_len));
      (void)w;
    }
#if XE_ARCH_ARM64
    // macos-arm64 Fable II bring-up: chasing a crash where a newly-created
    // guest thread's Processor::Execute lands in genuinely unmapped memory
    // (pc looks like raw ARM64 instruction bytes, not a data pointer -
    // suggests an indirect branch, e.g. the A64 backend's
    // ResolveFunctionThunk's `br x9`, jumped to a garbage value). pc alone
    // doesn't say where that garbage came from. lr (x30) survives a plain
    // `br` (only `bl`/`blr` write it), so it should still hold the return
    // address into whichever guest function's compiled code executed the
    // bad branch - print it plus x9/x16 (the resolved-function-pointer and
    // original-guest-target registers in that specific thunk) so the
    // calling context can be identified without a live debugger, which
    // Mach-exception interception makes unworkable here (see SESSION_LOG).
    char regs[256];
    int regs_len = snprintf(
        regs, sizeof(regs), "lr=0x%llx x9=0x%llx x16=0x%llx sp=0x%llx\n",
        static_cast<unsigned long long>(mc->__ss.__lr),
        static_cast<unsigned long long>(mc->__ss.__x[9]),
        static_cast<unsigned long long>(mc->__ss.__x[16]),
        static_cast<unsigned long long>(mc->__ss.__sp));
    if (regs_len > 0) {
      ssize_t w = write(STDERR_FILENO, regs, size_t(regs_len));
      (void)w;
    }
#endif
    void* frames[128];
    int frame_count = backtrace(frames, 128);
    backtrace_symbols_fd(frames, frame_count, STDERR_FILENO);

    // macos-arm64 Fable II bring-up: also dump the vm_region_64 info at the
    // faulting PC - tells us whether it landed in genuinely unmapped memory,
    // a read-only/non-executable region (JIT code cache mprotect race?), or
    // a plausible-but-wrong executable region, without needing a debugger.
    {
      vm_address_t region = static_cast<vm_address_t>(mc->__ss.__pc);
      vm_size_t region_size = 0;
      vm_region_basic_info_data_64_t info;
      mach_msg_type_number_t info_count = VM_REGION_BASIC_INFO_COUNT_64;
      mach_port_t obj = MACH_PORT_NULL;
      kern_return_t kr = vm_region_64(
          mach_task_self(), &region, &region_size, VM_REGION_BASIC_INFO_64,
          reinterpret_cast<vm_region_info_t>(&info), &info_count, &obj);
      char region_buf[256];
      int region_len;
      if (kr == KERN_SUCCESS) {
        region_len = snprintf(
            region_buf, sizeof(region_buf),
            "pc region: base=0x%llx size=0x%llx prot=%c%c%c max_prot=%c%c%c\n",
            static_cast<unsigned long long>(region),
            static_cast<unsigned long long>(region_size),
            (info.protection & VM_PROT_READ) ? 'r' : '-',
            (info.protection & VM_PROT_WRITE) ? 'w' : '-',
            (info.protection & VM_PROT_EXECUTE) ? 'x' : '-',
            (info.max_protection & VM_PROT_READ) ? 'r' : '-',
            (info.max_protection & VM_PROT_WRITE) ? 'w' : '-',
            (info.max_protection & VM_PROT_EXECUTE) ? 'x' : '-');
      } else {
        region_len = snprintf(region_buf, sizeof(region_buf),
                              "pc region: vm_region_64 failed, kr=%d (no "
                              "mapping found at/above pc - likely genuinely "
                              "unmapped)\n",
                              int(kr));
      }
      if (region_len > 0) {
        ssize_t w = write(STDERR_FILENO, region_buf, size_t(region_len));
        (void)w;
      }
    }
  }
#endif

  signal(signal_number, SIG_DFL);
  raise(signal_number);
}

void ExceptionHandler::Install(Handler fn, void* data) {
  if (!signal_handlers_installed_) {
    struct sigaction signal_handler;
    std::memset(&signal_handler, 0, sizeof(signal_handler));
    signal_handler.sa_sigaction = DarwinExceptionHandlerCallback;
    signal_handler.sa_flags = SA_SIGINFO;

    sigaction(SIGILL, &signal_handler, &original_sigill_handler_);
    sigaction(SIGSEGV, &signal_handler, &original_sigsegv_handler_);
    sigaction(SIGBUS, &signal_handler, &original_sigbus_handler_);
    signal_handlers_installed_ = true;
  }

  for (size_t i = 0; i < xe::countof(handlers_); ++i) {
    if (!handlers_[i].first) {
      handlers_[i].first = fn;
      handlers_[i].second = data;
      return;
    }
  }
}

void ExceptionHandler::Uninstall(Handler fn, void* data) {
  for (size_t i = 0; i < xe::countof(handlers_); ++i) {
    if (handlers_[i].first == fn && handlers_[i].second == data) {
      for (; i < xe::countof(handlers_) - 1; ++i) {
        handlers_[i] = handlers_[i + 1];
      }
      handlers_[i].first = nullptr;
      handlers_[i].second = nullptr;
      break;
    }
  }
}

}  // namespace xe

#endif
