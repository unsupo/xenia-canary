#pragma once

#include <cstdlib>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#if !defined(NDEBUG)
#define XE_ENABLE_PM4_DISASM 1
#endif

using namespace xe::gpu::xenos;
void COMMAND_PROCESSOR::ExecuteIndirectBuffer(uint32_t ptr,
                                              uint32_t count) XE_RESTRICT {
  SCOPE_profile_cpu_f("gpu");

  trace_writer_.WriteIndirectBufferStart(ptr, count * sizeof(uint32_t));
  if (count != 0) {
    RingBuffer old_reader = reader_;

    // Execute commands!
    new (&reader_)
        RingBuffer(memory_->TranslatePhysical(ptr), count * sizeof(uint32_t));
    reader_.set_write_offset(count * sizeof(uint32_t));
    // prefetch the wraparound range
    // it likely is already in L3 cache, but in a zen system it may be another
    // chiplets l3
    reader_.BeginPrefetchedRead<swcache::PrefetchTag::Level2>(
        COMMAND_PROCESSOR::GetCurrentRingReadCount());
    do {
      if (COMMAND_PROCESSOR::ExecutePacket()) {
        // A downstream WAIT_REG_MEM gave up on a CPU<->GPU standoff - stop
        // draining this IB and let it propagate to ExecutePrimaryBuffer.
        if (wrm_deadlock_abort_) {
          break;
        }
        continue;
      } else {
        // Return up a level if we encounter a bad packet. macos-arm64 Fable II
        // bring-up: no assert_always() (SIGTRAPs in this NDEBUG build) - stale
        // IB content lands here routinely; just unwind.
        break;
      }
    } while (reader_.read_count());

    trace_writer_.WriteIndirectBufferEnd();
    reader_ = old_reader;
  } else {
    // rare, but i've seen it happen! (and then a division by 0 occurs)
    return;
  }
}
XE_NOINLINE
static void LOGU32s(logging::LoggerBatch<LogLevel::Debug>& logger,
                    const std::vector<uint32_t>& values) {
  bool first = true;
#if 0
  XELOGD("[ ");
  for (auto&& val : values) {
    if (first) {
      XELOGD("0x{:08X}", val);
      first = false;
    } else {
      XELOGD(", 0x{:08X}", val);
    }
  }
  XELOGD(" ]");
#else

  for (auto&& val : values) {
    if (first) {
      logger("0x{:08X}", val);
      first = false;
    } else {
      logger(", 0x{:08X}", val);
    }
  }
#endif
}

std::string GenerateRegnameForPm4Print(uint32_t reg) {
  auto reg_info = RegisterFile::GetRegisterInfo(reg);

  if (reg_info) {
    return reg_info->name;
  } else {
    return fmt::format("Unknown_Reg_{:04X}", reg);
  }
}

XE_NOINLINE
void COMMAND_PROCESSOR::DisassembleCurrentPacket() XE_RESTRICT {
  xe::gpu::PacketInfo packet_info;

  logging::LoggerBatch<LogLevel::Debug> logger{};

  if (PacketDisassembler::DisasmPacket(reader_.buffer() + reader_.read_offset(),
                                       &packet_info)) {
    logger("CP - {}, count {}, predicated = {}\n", packet_info.type_info->name,
           packet_info.count, packet_info.predicated);

#define LOG_ACTION_FIELD(__type, name) \
  logger("\t" #name " = {:08X}\n", static_cast<uint32_t>(action.__type.name))
#define LOG_ACTION_FIELD_DEC(__type, name) \
  logger("\t" #name " = {}\n", static_cast<size_t>(action.__type.name))

#define LOG_ENDIANNESS(__type, name) \
  logger("\t" #name " = {}\n",       \
         xenos::GetEndianEnglishDescription(action.__type.name))
#define LOG_PRIMTYPE(__type, name) \
  logger("\t" #name " = {}\n",     \
         xenos::GetPrimitiveTypeEnglishDescription(action.__type.name))
    for (auto&& action : packet_info.actions) {
      using PType = PacketAction::Type;
      switch (action.type) {
        case PType::kRegisterWrite:
          break;
        case PType::kSetBinMask:
          logger("\tSetBinMask {}\n", action.set_bin_mask.value);
          break;
        case PType::kSetBinSelect:
          logger("\tSetBinSelect {}\n", action.set_bin_select.value);
          break;
        case PType::kMeInit:
          logger("\tMeInit - ");
          LOGU32s(logger, action.words);
          logger("\n");
          break;
        case PType::kGenInterrupt:
          logger("\tGenInterrupt for cpu mask {:04X}\n",
                 action.gen_interrupt.cpu_mask);
          break;
        case PType::kSetBinMaskHi:
        case PType::kSetBinMaskLo:
        case PType::kSetBinSelectHi:
        case PType::kSetBinSelectLo:
          LOG_ACTION_FIELD(lohi_op, value);
          break;
        case PType::kWaitRegMem:
          LOG_ACTION_FIELD(wait_reg_mem, wait_info);
          LOG_ACTION_FIELD(wait_reg_mem, poll_reg_addr);
          LOG_ACTION_FIELD(wait_reg_mem, ref);
          LOG_ACTION_FIELD(wait_reg_mem, mask);
          LOG_ACTION_FIELD(wait_reg_mem, wait);
          break;
        case PType::kRegRmw: {
          uint32_t rmw_info = action.reg_rmw.rmw_info;
          uint32_t and_mask = action.reg_rmw.and_mask;
          uint32_t or_mask = action.reg_rmw.or_mask;

          uint32_t and_mask_is_reg = (rmw_info >> 31) & 0x1;
          uint32_t or_mask_is_reg = (rmw_info >> 30) & 0x1;

          std::string and_mask_str;

          if (and_mask_is_reg) {
            and_mask_str = GenerateRegnameForPm4Print(and_mask & 0x1FFF);
          } else {
            and_mask_str = fmt::format("0x{:08X}", and_mask);
          }
          std::string or_mask_str;

          if (or_mask_is_reg) {
            or_mask_str = GenerateRegnameForPm4Print(or_mask & 0x1FFF);
          } else {
            or_mask_str = fmt::format("0x{:08X}", or_mask);
          }

          std::string dest = GenerateRegnameForPm4Print(rmw_info & 0x1FFF);

          logger("\t{} = ({} & {}) | {}\n", dest, dest, and_mask_str,
                 or_mask_str);
          LOG_ACTION_FIELD(reg_rmw, rmw_info);
          LOG_ACTION_FIELD(reg_rmw, and_mask);
          LOG_ACTION_FIELD(reg_rmw, or_mask);

          break;
        }
        case PType::kCondWrite:
          LOG_ACTION_FIELD(cond_write, wait_info);
          LOG_ACTION_FIELD(cond_write, poll_reg_addr);
          LOG_ACTION_FIELD(cond_write, ref);
          LOG_ACTION_FIELD(cond_write, mask);
          LOG_ACTION_FIELD(cond_write, write_reg_addr);
          LOG_ACTION_FIELD(cond_write, write_data);
          break;

        case PType::kEventWrite:
          LOG_ACTION_FIELD(event_write, initiator);
          break;
        case PType::kEventWriteSHD:
          LOG_ACTION_FIELD(event_write_shd, initiator);
          LOG_ACTION_FIELD(event_write_shd, address);
          LOG_ACTION_FIELD(event_write_shd, value);
          break;
        case PType::kEventWriteExt:
          LOG_ACTION_FIELD(event_write_ext, unk0);
          LOG_ACTION_FIELD(event_write_ext, unk1);
          break;
        case PType::kDrawIndx:
          LOG_ACTION_FIELD(draw_indx, dword0);
          LOG_ACTION_FIELD(draw_indx, dword1);
          LOG_ACTION_FIELD_DEC(draw_indx, index_count);
          LOG_PRIMTYPE(draw_indx, prim_type);
          LOG_ACTION_FIELD(draw_indx, src_sel);
          LOG_ACTION_FIELD(draw_indx, guest_base);
          LOG_ACTION_FIELD_DEC(draw_indx, index_size);
          LOG_ENDIANNESS(draw_indx, endianness);
          break;
        case PType::kDrawIndx2:
          LOG_ACTION_FIELD(draw_indx2, dword0);
          LOG_ACTION_FIELD_DEC(draw_indx2, index_count);
          LOG_PRIMTYPE(draw_indx2, prim_type);
          LOG_ACTION_FIELD(draw_indx2, src_sel);
          LOG_ACTION_FIELD_DEC(draw_indx2, indices_size);
          logger("Indices = ");
          LOGU32s(logger, action.words);
          logger("\n");
          break;
        case PType::kInvalidateState:
          LOG_ACTION_FIELD(invalidate_state, state_mask);
          break;
        case PType::kImLoad:
          LOG_ACTION_FIELD(im_load, shader_type);
          LOG_ACTION_FIELD(im_load, addr);
          LOG_ACTION_FIELD(im_load, start);
          LOG_ACTION_FIELD_DEC(im_load, size_dwords);
          break;
        case PType::kImLoadImmediate:
          LOG_ACTION_FIELD_DEC(im_load_imm, shader_type);
          LOG_ACTION_FIELD(im_load_imm, start);
          LOG_ACTION_FIELD_DEC(im_load_imm, size_dwords);
          logger("Shader instruction words = ");
          LOGU32s(logger, action.words);
          logger("\n");
          break;
        case PType::kWaitForIdle:
          LOG_ACTION_FIELD(wait_for_idle, probably_unused);
          break;
        case PType::kContextUpdate:
          LOG_ACTION_FIELD(context_update, maybe_unused);
          break;
        case PType::kVizQuery:
          LOG_ACTION_FIELD_DEC(vizquery, id);
          LOG_ACTION_FIELD_DEC(vizquery, end);
          LOG_ACTION_FIELD(vizquery, dword0);
          break;
        case PType::kEventWriteZPD:
          LOG_ACTION_FIELD(event_write_zpd, initiator);

          break;
        case PType::kMemWrite:
          LOG_ACTION_FIELD(mem_write, addr);
          LOG_ENDIANNESS(mem_write, endianness);
          logger("Values to write (with GpuSwap pre-applied) = ");
          LOGU32s(logger, action.words);
          logger("\n");
          break;
        case PType::kRegToMem:
          logger("{}\n", GenerateRegnameForPm4Print(action.reg2mem.reg_addr));
          LOG_ACTION_FIELD(reg2mem, mem_addr);
          LOG_ENDIANNESS(reg2mem, endianness);
          break;
        case PType::kIndirBuffer:
          LOG_ACTION_FIELD(indir_buffer, list_ptr);
          LOG_ACTION_FIELD_DEC(indir_buffer, list_length);
          break;
        case PType::kXeSwap:
          LOG_ACTION_FIELD(xe_swap, frontbuffer_ptr);
          break;
      }
    }
  } else {
    logger("Unknown packet! Failed to disassemble.\n");
  }
  logger.submit('d');
}
// macos-arm64 Fable II bring-up: rolling log of the last N PM4 packet headers
// (with the guest ring offset they were read from) so we can dump the exact
// stream leading into a stale-IB / bad-packet failure. XE_LOG_WRM-gated dump.
struct Pm4RecentEntry {
  uint32_t ring_off;
  uint32_t header;
  uint32_t w1;
  bool in_ib;
};
static thread_local Pm4RecentEntry g_pm4_recent[48] = {};
static thread_local uint32_t g_pm4_recent_pos = 0;
static thread_local bool g_pm4_in_ib = false;
static void Pm4DumpRecent(const char* why) {
  static const bool log_wrm = std::getenv("XE_LOG_WRM") != nullptr;
  if (!log_wrm) return;
  XELOGW("PM4-STREAM dump ({}): last 48 packets (oldest first)", why);
  for (uint32_t i = 0; i < 48; ++i) {
    const Pm4RecentEntry& e = g_pm4_recent[(g_pm4_recent_pos + i) % 48];
    if (!e.header) continue;
    uint32_t t = e.header >> 30;
    uint32_t op3 = (e.header >> 8) & 0x7F;
    uint32_t cnt = ((e.header >> 16) & 0x3FFF) + 1;
    XELOGW("  [{}] off={:06X} hdr={:08X} type={} op={:02X} count={} w1={:08X}",
           e.in_ib ? "IB" : "P ", e.ring_off, e.header, t, op3, cnt, e.w1);
  }
}

// macos-arm64 Fable II bring-up, Phase 11/12: IB backing-memory identity/
// lifetime diagnostic (XE_IM_LOAD_LIFECYCLE env-gated). Phase 10 found that
// waiting longer before an IM_LOAD_IMMEDIATE read made the known-corrupt
// 285-dword vertex shader WORSE, not better - ruling out a plain "producer
// hasn't finished writing yet" latency race. Phase 11's settle check (re-read
// the same bytes 300us/1300us after LoadShader already consumed them) found
// 0/3636 changes - so it's not a live torn read at load time either; by the
// time xenia reads it, the content is already stable. Phase 11 also found no
// corrupt hash ever equals a confirmed-good hash seen elsewhere, ruling out
// simple whole-buffer aliasing (reading someone else's complete valid
// shader). What's left: does the SAME guest address hold different content
// across separate dispatches, and how far apart (in load order) are those
// dispatches? Phase 11's 8-slot LRU was too small to catch this - addresses
// climb in large strides through what looks like a big linear pool, so
// repeats are separated by hundreds of other loads. This version tracks
// every distinct address for the life of the run instead.
struct Pm4ImLoadImmAddrEntry {
  uint64_t first_seq = 0;
  uint64_t first_hash = 0;
  uint64_t last_seq = 0;
  uint64_t last_hash = 0;
  uint32_t observe_count = 0;
};
static thread_local std::unordered_map<uint32_t, Pm4ImLoadImmAddrEntry>
    g_im_load_imm_addr_hist;
// Global hash->first-address registry: if a "corrupt" hash matches content
// xenia already loaded from a DIFFERENT address, that's a byte-for-byte
// stale/aliased read, not scrambled bytes.
static thread_local std::unordered_map<uint64_t, uint32_t>
    g_im_load_imm_hash_first_addr;
static thread_local uint64_t g_im_load_imm_seq = 0;
static thread_local uint32_t g_im_load_imm_prev_addr = 0;
static thread_local bool g_im_load_imm_has_prev_addr = false;

static void Pm4ImLoadImmDiagnose(uint32_t guest_addr, uint32_t ring_off,
                                  bool in_ib, uint32_t size_dwords,
                                  const std::vector<uint32_t>& snap0,
                                  const uint32_t* host_words, uint64_t hash) {
  uint64_t seq = g_im_load_imm_seq++;
  int64_t addr_delta =
      g_im_load_imm_has_prev_addr
          ? int64_t(guest_addr) - int64_t(g_im_load_imm_prev_addr)
          : 0;
  g_im_load_imm_prev_addr = guest_addr;
  g_im_load_imm_has_prev_addr = true;

  // Per-address history: has xenia loaded from this exact address before,
  // and did the content change since then?
  auto it = g_im_load_imm_addr_hist.find(guest_addr);
  if (it == g_im_load_imm_addr_hist.end()) {
    XELOGW(
        "IM_LOAD-LIFECYCLE seq={} addr={:08X} addr_delta={} {} "
        "ring_off={:06X} hash={:016X} first observed load at this address "
        "({} dwords)",
        seq, guest_addr, addr_delta, in_ib ? "IB" : "P ", ring_off, hash,
        size_dwords);
    Pm4ImLoadImmAddrEntry e;
    e.first_seq = e.last_seq = seq;
    e.first_hash = e.last_hash = hash;
    e.observe_count = 1;
    g_im_load_imm_addr_hist.emplace(guest_addr, e);
  } else {
    Pm4ImLoadImmAddrEntry& e = it->second;
    uint64_t seq_delta = seq - e.last_seq;
    if (hash != e.last_hash) {
      XELOGW(
          "IM_LOAD-LIFECYCLE seq={} addr={:08X} addr_delta={} {} "
          "ring_off={:06X} CONTENT CHANGED vs seq={} ({} loads ago, first "
          "seen seq={}): prev_hash={:016X} now_hash={:016X} first_hash_ever="
          "{:016X} observe_count={}",
          seq, guest_addr, addr_delta, in_ib ? "IB" : "P ", ring_off,
          e.last_seq, seq_delta, e.first_seq, e.last_hash, hash,
          e.first_hash, e.observe_count + 1);
    } else {
      XELOGW(
          "IM_LOAD-LIFECYCLE seq={} addr={:08X} addr_delta={} {} "
          "ring_off={:06X} hash={:016X} content IDENTICAL to seq={} ({} "
          "loads ago, observe_count={})",
          seq, guest_addr, addr_delta, in_ib ? "IB" : "P ", ring_off, hash,
          e.last_seq, seq_delta, e.observe_count + 1);
    }
    e.last_seq = seq;
    e.last_hash = hash;
    ++e.observe_count;
  }

  // Global hash registry: does this content match a payload xenia already
  // saw at a *different* address?
  auto hit = g_im_load_imm_hash_first_addr.find(hash);
  if (hit != g_im_load_imm_hash_first_addr.end() &&
      hit->second != guest_addr) {
    XELOGW(
        "IM_LOAD-LIFECYCLE seq={} addr={:08X} hash={:016X} ALIASES content "
        "first seen at addr={:08X} - byte-for-byte copy of a payload xenia "
        "already loaded from a different address",
        seq, guest_addr, hash, hit->second);
  } else if (hit == g_im_load_imm_hash_first_addr.end()) {
    g_im_load_imm_hash_first_addr.emplace(hash, guest_addr);
  }

  // Settle check (kept from Phase 11 - already gave a clean 0/3636 negative
  // result, keep monitoring it as the lifecycle table grows): re-read the
  // identical bytes ~300us/~1300us after LoadShader already consumed them.
  uint32_t settle_first_diff = UINT32_MAX;
  bool settle_changed_300us = false, settle_changed_1300us = false;
  xe::threading::Sleep(std::chrono::microseconds(300));
  for (uint32_t i = 0; i < size_dwords; ++i) {
    if (xe::load_and_swap<uint32_t>(host_words + i) != snap0[i]) {
      settle_changed_300us = true;
      if (settle_first_diff == UINT32_MAX) settle_first_diff = i;
    }
  }
  xe::threading::Sleep(std::chrono::microseconds(1000));
  for (uint32_t i = 0; i < size_dwords; ++i) {
    if (xe::load_and_swap<uint32_t>(host_words + i) != snap0[i]) {
      settle_changed_1300us = true;
      if (settle_first_diff == UINT32_MAX) settle_first_diff = i;
    }
  }
  if (settle_changed_300us || settle_changed_1300us) {
    XELOGW(
        "IM_LOAD-LIFECYCLE seq={} addr={:08X} hash={:016X} STILL BEING "
        "WRITTEN after load: changed@+300us={} changed@+1300us={} "
        "first_diff_dword={} - guest wrote to this address AFTER xenia "
        "already loaded from it",
        seq, guest_addr, hash, settle_changed_300us, settle_changed_1300us,
        settle_first_diff);
  }
}

// macos-arm64 Fable II bring-up, Phase 18: on-demand shader dump - given a
// target hash via XE_DUMP_SHADER_HASH (hex, no 0x prefix), dump the full
// ucode + every texture_binding's fetch_constant/dimension/predication the
// first time a vertex or pixel shader with that hash becomes active. Built
// to resolve the Phase 17 question: does the rect-list blit draw's pixel
// shader really expect a live texture at fetch constant slot 0 on every
// code path, or is that tfetch conditional/context-dependent in a way that
// would mean the "clobbered" register isn't actually read on that pass.
// Keyed on hash, valued on the last-seen texture_bindings().size() - dump
// again whenever that count changes (e.g. 0 pre-analysis -> N post-analysis)
// instead of only once ever, since the first occurrence of a freshly-loaded
// shader may be queried before ucode analysis has populated its bindings.
static thread_local std::unordered_map<uint64_t, size_t> g_dumped_shader_hashes;
static void Pm4DumpShaderIfTargeted(Shader* shader, const char* label) {
  if (!shader) return;
  static const char* target_hex = std::getenv("XE_DUMP_SHADER_HASH");
  if (!target_hex) return;
  uint64_t target = strtoull(target_hex, nullptr, 16);
  uint64_t hash = shader->ucode_data_hash();
  if (hash != target) return;
  size_t binding_count = shader->texture_bindings().size();
  auto it = g_dumped_shader_hashes.find(hash);
  if (it != g_dumped_shader_hashes.end() && it->second == binding_count) {
    return;  // Already dumped this hash at this binding count.
  }
  g_dumped_shader_hashes[hash] = binding_count;

  const auto& uc = shader->ucode_data();
  XELOGW("SHADER-DUMP ({}) hash={:016X} type={} dwords={}", label, hash,
         uint32_t(shader->type()), uc.size());
  for (size_t i = 0; i < uc.size(); i += 8) {
    size_t n = std::min<size_t>(size_t(8), uc.size() - i);
    std::string line;
    for (size_t j = 0; j < n; ++j) {
      line += fmt::format("{:08X} ", uc[i + j]);
    }
    XELOGW("  [{:04X}] {}", i, line);
  }
  XELOGW("SHADER-DUMP ({}) hash={:016X} {} texture binding(s):", label, hash,
         shader->texture_bindings().size());
  for (const auto& binding : shader->texture_bindings()) {
    const auto& fi = binding.fetch_instr;
    XELOGW(
        "  TEX-BINDING idx={} fetch_constant={} opcode={} dim={} "
        "is_predicated={} predicate_condition={} result_write_mask={:#06b} "
        "result_storage_target={}",
        binding.binding_index, binding.fetch_constant,
        fi.opcode_name ? fi.opcode_name : "?", uint32_t(fi.dimension),
        fi.is_predicated, fi.predicate_condition,
        fi.result.original_write_mask, uint32_t(fi.result.storage_target));
  }
}

bool COMMAND_PROCESSOR::ExecutePacket() {
#if XE_ENABLE_PM4_DISASM == 1
  if (cvars::disassemble_pm4 && logging::ShouldLog(LogLevel::Debug)) {
    COMMAND_PROCESSOR::DisassembleCurrentPacket();
  }
#endif
  uint32_t pkt_ring_off = uint32_t(reader_.read_offset());
  const uint32_t packet = reader_.ReadAndSwap<uint32_t>();
  const uint32_t packet_type = packet >> 30;
  {
    Pm4RecentEntry& e = g_pm4_recent[g_pm4_recent_pos];
    e.ring_off = pkt_ring_off;
    e.header = packet;
    e.w1 = reader_.read_count() >= 4
               ? xe::load_and_swap<uint32_t>(
                     reinterpret_cast<const void*>(reader_.read_ptr()))
               : 0;
    e.in_ib = g_pm4_in_ib;
    g_pm4_recent_pos = (g_pm4_recent_pos + 1) % 48;
  }

  XE_LIKELY_IF(packet && packet != 0x0BADF00D) {
    XE_LIKELY_IF((packet != 0xCDCDCDCD)) {
    actually_execute_packet:
      // chrispy: reorder checks by probability
      XE_LIKELY_IF(packet_type == 3) {
        return COMMAND_PROCESSOR::ExecutePacketType3(packet);
      }
      else {
        if (packet_type ==
            0) {  // dont know whether 0 or 1 are the next most frequent
          return COMMAND_PROCESSOR::ExecutePacketType0(packet);
        } else {
          if (packet_type == 1) {
            return COMMAND_PROCESSOR::ExecutePacketType1(packet);
          } else {
            // originally there was a default case that msvc couldn't optimize
            // away because it doesnt have value range analysis but in reality
            // there is no default, a uint32_t >> 30 only has 4 possible values
            // and all are covered here
            // return COMMAND_PROCESSOR::ExecutePacketType2(packet);
            // executepackettype2 is identical
            goto handle_bad_packet;
          }
        }
      }
    }
    else {
      XELOGW("GPU packet is CDCDCDCD - probably read uninitialized memory!");
      goto actually_execute_packet;
    }
  }
  else {
  handle_bad_packet:
    trace_writer_.WritePacketStart(uint32_t(reader_.read_ptr() - 4), 1);
    trace_writer_.WritePacketEnd();
    return true;
  }
}
XE_NOINLINE
XE_COLD
bool COMMAND_PROCESSOR::ExecutePacketType0_CountOverflow(uint32_t count) {
  static std::atomic<uint32_t> warn{0};
  if ((warn++ & 0x3FF) == 0) {
    XELOGE(
        "ExecutePacketType0 overflow (read count {:08X}, packet count {:08X})",
        COMMAND_PROCESSOR::GetCurrentRingReadCount(), count * sizeof(uint32_t));
    Pm4DumpRecent("Type0 count overflow");
  }
  return false;
}
/*
    Todo: optimize this function this one along with execute packet type III are
   the most frequently called functions for PM4
*/
XE_NOINLINE
bool COMMAND_PROCESSOR::ExecutePacketType0(uint32_t packet) XE_RESTRICT {
  // Type-0 packet.
  // Write count registers in sequence to the registers starting at
  // (base_index << 2).

  uint32_t count = ((packet >> 16) & 0x3FFF) + 1;

  if (COMMAND_PROCESSOR::GetCurrentRingReadCount() >=
      count * sizeof(uint32_t)) {
    uint32_t base_index = (packet & 0x7FFF);
    uint32_t write_one_reg = (packet >> 15) & 0x1;

    // macos-arm64 Fable II bring-up: a Type0 whose base register is past the
    // end of the register file is stale-IB garbage (seen: base 0x7100).
    // Executing it is hundreds of rejected out-of-bounds WriteRegister calls
    // per frame - a real slice of the lag. Unwind to the caller instead.
    if (base_index >= RegisterFile::kRegisterCount) {
      static std::atomic<uint32_t> warn{0};
      if ((warn++ & 0x3FF) == 0) {
        XELOGW(
            "PM4 Type0: base register {:X} past the register file - stale IB "
            "content, unwinding.",
            base_index);
        Pm4DumpRecent("Type0 base register OOB");
      }
      return false;
    }

    trace_writer_.WritePacketStart(uint32_t(reader_.read_ptr() - 4), 1 + count);

    if (!write_one_reg) {
      COMMAND_PROCESSOR::WriteRegisterRangeFromRing(&reader_, base_index,
                                                    count);

    } else {
      COMMAND_PROCESSOR::WriteOneRegisterFromRing(base_index, count);
    }

    trace_writer_.WritePacketEnd();
    return true;
  } else {
    return COMMAND_PROCESSOR::ExecutePacketType0_CountOverflow(count);
  }
}
XE_NOINLINE
bool COMMAND_PROCESSOR::ExecutePacketType1(uint32_t packet) XE_RESTRICT {
  // Type-1 packet.
  // Contains two registers of data. Type-0 should be more common.
  trace_writer_.WritePacketStart(uint32_t(reader_.read_ptr() - 4), 3);
  uint32_t reg_index_1 = packet & 0x7FF;
  uint32_t reg_index_2 = (packet >> 11) & 0x7FF;
  uint32_t reg_data_1 = reader_.ReadAndSwap<uint32_t>();
  uint32_t reg_data_2 = reader_.ReadAndSwap<uint32_t>();
  COMMAND_PROCESSOR::WriteRegister(reg_index_1, reg_data_1);
  COMMAND_PROCESSOR::WriteRegister(reg_index_2, reg_data_2);
  trace_writer_.WritePacketEnd();
  return true;
}

bool COMMAND_PROCESSOR::ExecutePacketType2(uint32_t packet) XE_RESTRICT {
  // Type-2 packet.
  // No-op. Do nothing.
  trace_writer_.WritePacketStart(uint32_t(reader_.read_ptr() - 4), 1);
  trace_writer_.WritePacketEnd();
  return true;
}
XE_FORCEINLINE
XE_NOALIAS
uint32_t COMMAND_PROCESSOR::GetCurrentRingReadCount() {
  return reader_.read_count();
}
XE_NOINLINE
XE_COLD
bool COMMAND_PROCESSOR::ExecutePacketType3_CountOverflow(uint32_t count) {
  static std::atomic<uint32_t> warn{0};
  if ((warn++ & 0x3FF) == 0) {
    XELOGE(
        "ExecutePacketType3 overflow (read count {:08X}, packet count {:08X})",
        COMMAND_PROCESSOR::GetCurrentRingReadCount(), count * sizeof(uint32_t));
    Pm4DumpRecent("Type3 count overflow");
  }
  return false;
}
XE_NOINLINE
bool COMMAND_PROCESSOR::ExecutePacketType3(uint32_t packet) XE_RESTRICT {
  // Type-3 packet.
  uint32_t opcode = (packet >> 8) & 0x7F;
  uint32_t count = ((packet >> 16) & 0x3FFF) + 1;
  auto data_start_offset = reader_.read_offset();

  if (COMMAND_PROCESSOR::GetCurrentRingReadCount() >=
      count * sizeof(uint32_t)) {
    // To handle nesting behavior when tracing we special case indirect buffers.
    if (opcode == PM4_INDIRECT_BUFFER) {
      trace_writer_.WritePacketStart(uint32_t(reader_.read_ptr() - 4), 2);
    } else {
      trace_writer_.WritePacketStart(uint32_t(reader_.read_ptr() - 4),
                                     1 + count);
    }

    // & 1 == predicate - when set, we do bin check to see if we should execute
    // the packet. Only type 3 packets are affected.
    // We also skip predicated swaps, as they are never valid (probably?).
    if (packet & 1) {
      bool any_pass = (bin_select_ & bin_mask_) != 0;
      if (!any_pass || opcode == PM4_XE_SWAP) {
        reader_.AdvanceRead(count * sizeof(uint32_t));
        trace_writer_.WritePacketEnd();
        return true;
      }
    }

    bool result = false;
    switch (opcode) {
      case PM4_ME_INIT:
        result = COMMAND_PROCESSOR::ExecutePacketType3_ME_INIT(packet, count);
        break;
      case PM4_NOP:
        result = COMMAND_PROCESSOR::ExecutePacketType3_NOP(packet, count);
        break;
      case PM4_INTERRUPT:
        result = COMMAND_PROCESSOR::ExecutePacketType3_INTERRUPT(packet, count);
        break;
      case PM4_XE_SWAP:
        result = COMMAND_PROCESSOR::ExecutePacketType3_XE_SWAP(packet, count);
        break;
      case PM4_INDIRECT_BUFFER:
      case PM4_INDIRECT_BUFFER_PFD:
        result = COMMAND_PROCESSOR::ExecutePacketType3_INDIRECT_BUFFER(packet,
                                                                       count);
        break;
      case PM4_WAIT_REG_MEM:
        result =
            COMMAND_PROCESSOR::ExecutePacketType3_WAIT_REG_MEM(packet, count);
        break;
      case PM4_REG_RMW:
        result = COMMAND_PROCESSOR::ExecutePacketType3_REG_RMW(packet, count);
        break;
      case PM4_REG_TO_MEM:
        result =
            COMMAND_PROCESSOR::ExecutePacketType3_REG_TO_MEM(packet, count);
        break;
      case PM4_MEM_WRITE:
        result = COMMAND_PROCESSOR::ExecutePacketType3_MEM_WRITE(packet, count);
        break;
      case PM4_COND_WRITE:
        result =
            COMMAND_PROCESSOR::ExecutePacketType3_COND_WRITE(packet, count);
        break;
      case PM4_EVENT_WRITE:
        result =
            COMMAND_PROCESSOR::ExecutePacketType3_EVENT_WRITE(packet, count);
        break;
      case PM4_EVENT_WRITE_SHD:
        result = COMMAND_PROCESSOR::ExecutePacketType3_EVENT_WRITE_SHD(packet,
                                                                       count);
        break;
      case PM4_EVENT_WRITE_EXT:
        result = COMMAND_PROCESSOR::ExecutePacketType3_EVENT_WRITE_EXT(packet,
                                                                       count);
        break;
      case PM4_EVENT_WRITE_ZPD:
        result = COMMAND_PROCESSOR::ExecutePacketType3_EVENT_WRITE_ZPD(packet,
                                                                       count);
        break;
      case PM4_DRAW_INDX:
        result = COMMAND_PROCESSOR::ExecutePacketType3_DRAW_INDX(packet, count);
        break;
      case PM4_DRAW_INDX_2:
        result =
            COMMAND_PROCESSOR::ExecutePacketType3_DRAW_INDX_2(packet, count);
        break;
      case PM4_SET_CONSTANT:
        result =
            COMMAND_PROCESSOR::ExecutePacketType3_SET_CONSTANT(packet, count);
        break;
      case PM4_SET_CONSTANT2:
        result =
            COMMAND_PROCESSOR::ExecutePacketType3_SET_CONSTANT2(packet, count);
        break;
      case PM4_LOAD_ALU_CONSTANT:
        result = COMMAND_PROCESSOR::ExecutePacketType3_LOAD_ALU_CONSTANT(packet,
                                                                         count);
        break;
      case PM4_SET_SHADER_CONSTANTS:
        result = COMMAND_PROCESSOR::ExecutePacketType3_SET_SHADER_CONSTANTS(
            packet, count);
        break;
      case PM4_IM_LOAD:
        result = COMMAND_PROCESSOR::ExecutePacketType3_IM_LOAD(packet, count);
        break;
      case PM4_IM_LOAD_IMMEDIATE:
        result = COMMAND_PROCESSOR::ExecutePacketType3_IM_LOAD_IMMEDIATE(packet,
                                                                         count);
        break;
      case PM4_INVALIDATE_STATE:
        result = COMMAND_PROCESSOR::ExecutePacketType3_INVALIDATE_STATE(packet,
                                                                        count);
        break;
      case PM4_VIZ_QUERY:
        result = COMMAND_PROCESSOR::ExecutePacketType3_VIZ_QUERY(packet, count);
        break;

      case PM4_SET_BIN_MASK_LO: {
        uint32_t value = reader_.ReadAndSwap<uint32_t>();
        bin_mask_ = (bin_mask_ & 0xFFFFFFFF00000000ull) | value;
        result = true;
      } break;
      case PM4_SET_BIN_MASK_HI: {
        uint32_t value = reader_.ReadAndSwap<uint32_t>();
        bin_mask_ =
            (bin_mask_ & 0xFFFFFFFFull) | (static_cast<uint64_t>(value) << 32);
        result = true;
      } break;
      case PM4_SET_BIN_SELECT_LO: {
        uint32_t value = reader_.ReadAndSwap<uint32_t>();
        bin_select_ = (bin_select_ & 0xFFFFFFFF00000000ull) | value;
        result = true;
      } break;
      case PM4_SET_BIN_SELECT_HI: {
        uint32_t value = reader_.ReadAndSwap<uint32_t>();
        bin_select_ = (bin_select_ & 0xFFFFFFFFull) |
                      (static_cast<uint64_t>(value) << 32);
        result = true;
      } break;
      case PM4_SET_BIN_MASK: {
        assert_true(count == 2);
        uint64_t val_hi = reader_.ReadAndSwap<uint32_t>();
        uint64_t val_lo = reader_.ReadAndSwap<uint32_t>();
        bin_mask_ = (val_hi << 32) | val_lo;
        result = true;
      } break;
      case PM4_SET_BIN_SELECT: {
        assert_true(count == 2);
        uint64_t val_hi = reader_.ReadAndSwap<uint32_t>();
        uint64_t val_lo = reader_.ReadAndSwap<uint32_t>();
        bin_select_ = (val_hi << 32) | val_lo;
        result = true;
      } break;
      case PM4_CONTEXT_UPDATE: {
        assert_true(count == 1);
        uint32_t value = reader_.ReadAndSwap<uint32_t>();
        XELOGGPU("GPU context update = {:08X}", value);
        assert_true(value == 0);
        result = true;
        break;
      }
      case PM4_WAIT_FOR_IDLE: {
        // This opcode is used by 5454084E while going / being ingame.
        assert_true(count == 1);
        uint32_t value = reader_.ReadAndSwap<uint32_t>();
        XELOGGPU("GPU wait for idle = {:08X}", value);
        result = true;
        break;
      }

      default:
        return COMMAND_PROCESSOR::HitUnimplementedOpcode(opcode, count);
    }

    trace_writer_.WritePacketEnd();
#if XE_ENABLE_TRACE_WRITER_INSTRUMENTATION == 1

    if (opcode == PM4_XE_SWAP) {
      // End the trace writer frame.
      if (trace_writer_.is_open()) {
        trace_writer_.WriteEvent(EventCommand::Type::kSwap);
        trace_writer_.Flush();
        if (trace_state_ == TraceState::kSingleFrame) {
          trace_state_ = TraceState::kDisabled;
          trace_writer_.Close();
        }
      } else if (trace_state_ == TraceState::kSingleFrame) {
        // New trace request - we only start tracing at the beginning of a
        // frame.
        uint32_t title_id = kernel_state_->GetExecutableModule()->title_id();
        auto file_name = fmt::format("{:08X}_{}.xtr", title_id, counter_ - 1);
        auto path = trace_frame_path_ / file_name;
        trace_writer_.Open(path, title_id);
        InitializeTrace();
      }
    }
#endif

    assert_true(reader_.read_offset() ==
                (data_start_offset + (count * sizeof(uint32_t))) %
                    reader_.capacity());
    return result;
  } else {
    return COMMAND_PROCESSOR::ExecutePacketType3_CountOverflow(count);
  }
}

XE_NOINLINE
XE_COLD
bool COMMAND_PROCESSOR::HitUnimplementedOpcode(uint32_t opcode,
                                               uint32_t count) XE_RESTRICT {
  // macos-arm64 Fable II bring-up: PM4 desyncs (stale IB content dispatched
  // before the D3D producer filled it) reach this path on garbage opcodes.
  // Don't assert_always() (it SIGTRAPs even in this NDEBUG build) and don't
  // AdvanceRead a bogus count - just unwind to the caller, which recovers at
  // the ring head.
  static std::atomic<uint32_t> warn{0};
  if ((warn++ & 0xFF) == 0) {
    XELOGGPU("Unimplemented GPU OPCODE: 0x{:02X} COUNT: {}", opcode, count);
  }
  trace_writer_.WritePacketEnd();
  return false;
}
XE_NOINLINE
bool COMMAND_PROCESSOR::ExecutePacketType3_ME_INIT(uint32_t packet,
                                                   uint32_t count) XE_RESTRICT {
  // initialize CP's micro-engine
  me_bin_.resize(count);
  for (uint32_t i = 0; i < count; i++) {
    me_bin_[i] = reader_.ReadAndSwap<uint32_t>();
  }
  return true;
}

bool COMMAND_PROCESSOR::ExecutePacketType3_NOP(uint32_t packet,
                                               uint32_t count) XE_RESTRICT {
  // skip N 32-bit words to get to the next packet
  // No-op, ignore some data.
  reader_.AdvanceRead(count * sizeof(uint32_t));
  return true;
}
XE_NOINLINE
bool COMMAND_PROCESSOR::ExecutePacketType3_INTERRUPT(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  SCOPE_profile_cpu_f("gpu");

  // generate interrupt from the command stream
  uint32_t cpu_mask = reader_.ReadAndSwap<uint32_t>();
  register_file_->values[XE_GPU_REG_CP_INT_STATUS] |= 1 | (cpu_mask << 8);
  for (int n = 0; n < 6; n++) {
    if (cpu_mask & (1 << n)) {
      graphics_system_->DispatchInterruptCallback(1, n);
    }
  }
  return true;
}
XE_NOINLINE
bool COMMAND_PROCESSOR::ExecutePacketType3_XE_SWAP(uint32_t packet,
                                                   uint32_t count) XE_RESTRICT {
  SCOPE_profile_cpu_f("gpu");

  Profiler::Flip();

  // Xenia-specific VdSwap hook.
  // VdSwap will post this to tell us we need to swap the screen/fire an
  // interrupt.
  // 63 words here, but only the first has any data.
  uint32_t magic = reader_.ReadAndSwap<fourcc_t>();
  assert_true(magic == kSwapSignature);

  // TODO(benvanik): only swap frontbuffer ptr.
  uint32_t frontbuffer_ptr = reader_.ReadAndSwap<uint32_t>();
  uint32_t frontbuffer_width = reader_.ReadAndSwap<uint32_t>();
  uint32_t frontbuffer_height = reader_.ReadAndSwap<uint32_t>();
  reader_.AdvanceRead((count - 4) * sizeof(uint32_t));

  COMMAND_PROCESSOR::IssueSwap(frontbuffer_ptr, frontbuffer_width,
                               frontbuffer_height);

  ++counter_;
  return true;
}

bool COMMAND_PROCESSOR::ExecutePacketType3_INDIRECT_BUFFER(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  // indirect buffer dispatch
  uint32_t list_ptr = CpuToGpu(reader_.ReadAndSwap<uint32_t>());
  uint32_t list_length = reader_.ReadAndSwap<uint32_t>();
  assert_zero(list_length & ~0xFFFFF);
  list_length &= 0xFFFFF;

  // macos-arm64 Fable II bring-up: the D3D producer sometimes advances the
  // ring write pointer past an IB-dispatch packet before it has finished
  // filling that IB, pointing us at stale/uninitialized command memory. Skip a
  // dispatch whose length is wild or whose first word isn't a plausible PM4
  // header rather than parsing megabytes of garbage (and wedging on a bogus
  // WAIT_REG_MEM buried in it).
  const uint32_t cpu_ptr = GpuToCpu(list_ptr);
  bool plausible = list_length != 0 && list_length <= 0x20000 &&
                   cpu_ptr >= 0x1000 && cpu_ptr < 0x20000000;
  if (plausible) {
    uint32_t w0 =
        xe::load_and_swap<uint32_t>(memory_->TranslatePhysical(cpu_ptr));
    uint32_t t0 = w0 >> 30;
    plausible = (t0 == 0 || t0 == 3) && w0 != 0xFFFFFFFF;
  }
  if (!plausible) {
    static std::atomic<uint32_t> warn{0};
    if ((warn++ & 0x7F) == 0) {
      XELOGW(
          "PM4 INDIRECT_BUFFER: skipping an implausible dispatch (ptr {:08X}, "
          "length {:05X}) - stale ring content.",
          cpu_ptr, list_length);
      Pm4DumpRecent("implausible INDIRECT_BUFFER");
    }
    return true;
  }
  bool prev_in_ib = g_pm4_in_ib;
  g_pm4_in_ib = true;
  COMMAND_PROCESSOR::ExecuteIndirectBuffer(cpu_ptr, list_length);
  g_pm4_in_ib = prev_in_ib;
  return true;
}

/*
        chrispy: this is fine to inline, as a noinline function it compiled down
   to 54 bytes
*/
static bool MatchValueAndRef(uint32_t value, uint32_t ref, uint32_t wait_info) {
  // smaller code is generated than the #else path, although whether it is
  // faster i do not know. i don't think games do an enormous number of
  // cond_write though, so we have picked the path with the smaller codegen. we
  // do technically have more instructions executed vs the switch case method,
  // but we have no mispredicts and most of our instructions are 0.25/0.3
  // throughput
  return ((((value < ref) << 1) | ((value <= ref) << 2) |
           ((value == ref) << 3) | ((value != ref) << 4) |
           ((value >= ref) << 5) | ((value > ref) << 6) | (1 << 7)) >>
          (wait_info & 7)) &
         1;
}
XE_NOINLINE
bool COMMAND_PROCESSOR::ExecutePacketType3_WAIT_REG_MEM(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  SCOPE_profile_cpu_f("gpu");

  // wait until a register or memory location is a specific value
  uint32_t wait_info = reader_.ReadAndSwap<uint32_t>();
  uint32_t poll_reg_addr = reader_.ReadAndSwap<uint32_t>();
  uint32_t ref = reader_.ReadAndSwap<uint32_t>();
  uint32_t mask = reader_.ReadAndSwap<uint32_t>();
  uint32_t wait = reader_.ReadAndSwap<uint32_t>();

  bool is_memory = (wait_info & 0x10) != 0;
  assert_true(is_memory || poll_reg_addr < RegisterFile::kRegisterCount);

  {
    static const bool log_wrm = std::getenv("XE_LOG_WRM") != nullptr;
    if (log_wrm) {
      uint32_t fn = wait_info & 0x7;  // 0=always 1=< 2=<= 3=== 4=!= 5=>= 6=>
      const char* fns[] = {"ALWAYS", "<", "<=", "==", "!=", ">=", ">", "?7"};
      uint32_t cur = 0;
      if (is_memory) {
        cur = *reinterpret_cast<uint32_t*>(
            memory_->TranslatePhysical(poll_reg_addr & ~uint32_t(0x3)));
        cur = xenos::GpuSwap(cur,
                             static_cast<xenos::Endian>(poll_reg_addr & 0x3));
      } else {
        cur = register_file_->values[poll_reg_addr];
      }
      XELOGW(
          "WRM-TRACE WAIT_REG_MEM space={} addr={:08X} ref={:08X} mask={:08X} "
          "fn={} wait_info={:08X} | cur={:08X} (cur&mask={:08X})",
          is_memory ? "MEM" : "REG", poll_reg_addr, ref, mask, fns[fn],
          wait_info, cur, cur & mask);
    }
  }

  // macos-arm64 Fable II bring-up: an IB dispatched before its D3D producer
  // filled it is parsed as garbage, and the garbage words routinely decode to
  // a WAIT_REG_MEM whose poll target is unsatisfiable - a physical address far
  // outside guest RAM, or mask==0 against a non-zero ref with an equality
  // function (value & 0 can only ever equal 0). Spinning 4s on each of these
  // before the wall-clock breaker trips wedges the CP for minutes. Detect the
  // impossible poll up front and bail straight to the ring head.
  if (is_memory) {
    uint32_t phys = poll_reg_addr & ~uint32_t(0x3);
    // A real Fable II / Halo 3 memory WAIT_REG_MEM polls only the GPU
    // register/fence aperture (system command buffer, ring buffers, the
    // 0x1FC81xxx fence page, the 0x1FC800xx CP<->CPU mailbox) - all in
    // 0x1FC00000-0x1FDFFFFF and always mapped. Garbage decoded from a stale IB
    // routinely yields a poll address elsewhere, and even a "plausible" one
    // (e.g. 0x1F504000 in the on-demand D3D command pool, or a guest-code
    // address like 0x82504000) whose page xenia has not committed - so the very
    // first `value = value_ref` read below SIGSEGVs. Anything outside that
    // aperture, or an unsatisfiable compare (mask==0 vs a non-zero ref under
    // equality), is stale content: bail to the ring head instead of reading it.
    // Tightest always-mapped bound: the mailbox / fence page / ring-buffer
    // cluster (VdInitializeRingBuffer uses 0x1FC82000 and 0x1FC9A000). Every
    // observed real poll is at 0x1FC800xx or 0x1FC81xxx.
    bool addr_ok = phys >= 0x1FC80000u && phys < 0x1FCA0000u;
    bool satisfiable =
        addr_ok && (mask != 0 || MatchValueAndRef(0, ref, wait_info));
    if (!satisfiable) {
      static std::atomic<uint32_t> warn{0};
      if ((warn++ & 0x3F) == 0) {
        XELOGW(
            "PM4 WAIT_REG_MEM: unsatisfiable poll (addr={:08X} ref={:08X} "
            "mask={:08X} wait_info={:08X}) - stale IB content, abandoning the "
            "pending ring.",
            poll_reg_addr, ref, mask, wait_info);
      }
      wrm_deadlock_abort_ = true;
      return false;
    }
  }

  const volatile uint32_t& value_ref =
      is_memory ? *reinterpret_cast<uint32_t*>(memory_->TranslatePhysical(
                      poll_reg_addr & ~uint32_t(0x3)))
                : register_file_->values[poll_reg_addr];

  bool matched = false;

  // macos-arm64: an unsatisfied memory WAIT_REG_MEM means the worker has
  // consumed every packet it can until the guest writes the polled value.
  // Fable II's GPU-management thread treats "GPU parked here" as "GPU drained"
  // for its flush/pump loop, and its frame-pacing fence only advances once
  // that loop completes - so publish this state so the fence publisher can
  // release it (see GraphicsSystem::PublishGpuIdentifier / is_ring_idle()).
  uint32_t wrm_spins = 0;

  // macos-arm64 (16 KB host pages): the guest reaches the CP<->CPU mailbox
  // (guest phys 0x1FC800xx) through an 0xE0000000-window pointer, and the A64
  // JIT shifts every >= 0xE0000000 access +0x1000 to clear the host guard page
  // (a64_seq_util.h ComputeMemoryAddress / GraphicsSystem::GuestU32Host). A
  // store the guest believes lands at phys P lands at P + 0x1000, so this poll
  // of the unshifted P never observes it. Also read P + 0x1000. ref here is a
  // specific non-zero token, so a coincidental alias match is not a concern.
  const volatile uint32_t* value_ref_alias = nullptr;
  // The CP<->CPU mailbox page. A poll here is the handshake that has no async
  // GPU to complete it on this port (nothing - guest or PM4 - writes 0x1FC800xx;
  // it reads xenia's ring poison 0x0BADF00D forever). See the synthesis path in
  // the spin loop below.
  bool poll_is_mailbox = false;
  if (is_memory) {
    uint32_t phys = poll_reg_addr & ~uint32_t(0x3);
    if (phys >= 0x1FC80000 && phys < 0x1FC81000) {
      value_ref_alias = reinterpret_cast<uint32_t*>(
          memory_->TranslatePhysical(phys + 0x1000));
      poll_is_mailbox = true;
    }
  }
  const uint64_t wrm_entry_ms = uint64_t(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());

  do {
    uint32_t value = value_ref;
    if (is_memory) {
      trace_writer_.WriteMemoryRead(CpuToGpu(poll_reg_addr & ~uint32_t(0x3)),
                                    sizeof(uint32_t));
      value = xenos::GpuSwap(value,
                             static_cast<xenos::Endian>(poll_reg_addr & 0x3));
    } else {
      if (poll_reg_addr == XE_GPU_REG_COHER_STATUS_HOST) {
        MakeCoherent();
        value = value_ref;
      }
    }
    matched = MatchValueAndRef(value & mask, ref, wait_info);
    if (!matched && value_ref_alias) {
      uint32_t av = xenos::GpuSwap(
          *value_ref_alias, static_cast<xenos::Endian>(poll_reg_addr & 0x3));
      if (MatchValueAndRef(av & mask, ref, wait_info)) {
        if (wrm_spins > 100) {
          XELOGW(
              "PM4 WAIT_REG_MEM addr={:08X} ref={:08X}: matched via +0x1000 "
              "JIT-shift alias after {} spins.",
              poll_reg_addr, ref, wrm_spins);
        }
        matched = true;
      }
    }

    if (!matched) {
      wrm_spins++;

      // Wait.
      if (is_memory && !wait_reg_mem_parked_.load(std::memory_order_relaxed)) {
        wait_reg_mem_parked_.store(true, std::memory_order_relaxed);
        if (graphics_system_) {
          graphics_system_->PublishGpuIdentifier();
        }
      }

      // Execute pending host functions (e.g. CallInThread callbacks - the
      // VdSwap IssueSwap dispatch) so the CP thread doesn't starve them while
      // stalled here.
      for (;;) {
        std::function<void()> fn;
        {
          std::lock_guard<std::mutex> lock(pending_fns_mutex_);
          if (pending_fns_.empty()) {
            break;
          }
          fn = std::move(pending_fns_.front());
          pending_fns_.pop();
        }
        fn();
      }

      // macos-arm64 Fable II bring-up diagnostic: surface what this poll is
      // actually stuck on the first time it spins past ~0.5s.
      if (wrm_spins == 5000) {
        XELOGW(
            "PM4 WAIT_REG_MEM stalling: is_memory={} addr={:08X} ref={:08X} "
            "mask={:08X} wait_info={:08X} wait={:08X} cur={:08X}",
            is_memory, poll_reg_addr, ref, mask, wait_info, wait, value);
      }

      // macos-arm64 Fable II: CP<->CPU mailbox synthesis. The guest submits an
      // IB containing this WAIT_REG_MEM on 0x1FC800xx and then parks its own
      // producer/pump thread waiting for the GPU to drain - but the thing that
      // would release this poll (a CP-microcode mailbox write on real hardware)
      // is not emulated, and no guest store ever targets it either, so the two
      // sides deadlock with no async GPU to break it. After ~1.2s stuck on a
      // mailbox slot, write the value the guest told us to wait for straight
      // into the slot and treat the wait as satisfied: downstream packets that
      // consume the token then read a sane value (ref is a valid guest pointer
      // or a small count), and the CP proceeds to the batch's CP_INTERRUPT,
      // which fires the ISR that releases the guest's pump loop. This keeps the
      // IB's draws (unlike the whole-ring abandon fallback below).
      if (poll_is_mailbox) {
        uint64_t now_ms = uint64_t(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        if (now_ms - wrm_entry_ms > 50) {
          uint32_t* slot = reinterpret_cast<uint32_t*>(
              memory_->TranslatePhysical(poll_reg_addr & ~uint32_t(0x3)));
          *slot = xenos::GpuSwap(
              ref, static_cast<xenos::Endian>(poll_reg_addr & 0x3));
          static std::atomic<uint32_t> warn{0};
          if ((warn++ & 0x1F) == 0) {
            XELOGW(
                "PM4 WAIT_REG_MEM addr={:08X}: CP<->CPU mailbox never written "
                "({} ms) - synthesising ref={:08X} and continuing.",
                poll_reg_addr, now_ms - wrm_entry_ms, ref);
          }
          matched = true;
          if (wait_reg_mem_parked_.load(std::memory_order_relaxed)) {
            wait_reg_mem_parked_.store(false, std::memory_order_relaxed);
            if (graphics_system_) {
              graphics_system_->PublishGpuIdentifier();
            }
          }
          break;
        }
      }

      // CPU<->GPU handshake deadlock breaker (wall-clock). This WAIT_REG_MEM
      // often deadlocks by *churning* - re-entering, matching, stalling again -
      // rather than one long spin, so a per-call spin count never trips. Track
      // real forward progress (counter_ advancing) instead: if the worker has
      // made none for ~2.5s while stuck here, abandon the pending ring to the
      // write pointer. The guest re-submits, the CP reaches the batch's
      // CP_INTERRUPT, and the source-1 ISR releases the guest's pump loop.
      // Applies to register polls too (a register the guest's ISR would set,
      // never reached because the ISR can't run while we hold the CP thread).
      {
        uint64_t now_ms = uint64_t(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        // "Progress" = the guest kicked off a meaningful batch of new ring
        // work. A trickle of 1-2 submissions while otherwise wedged doesn't
        // count (the guest producer re-issuing the same stuck batch). If real
        // progress has stalled for ~2s while we're stuck here, stop waiting
        // and run the rest of this buffer (its CP_INTERRUPT releases the
        // guest's pump loop).
        // Genuine forward progress = the guest kicked off a substantial batch
        // of new ring work (not the 1-2/frame it re-issues while wedged, and
        // not IssueSwap which keeps running from pending_fns_ during the
        // stall).
        uint32_t cnt = kickoff_count_.load(std::memory_order_relaxed);
        if (wrm_last_progress_ms_ == 0 ||
            cnt - wrm_last_progress_counter_ >= 32) {
          wrm_last_progress_counter_ = cnt;
          wrm_last_progress_ms_ = now_ms;
        }
        // Abandon if no real batch for ~4s while stuck here.
        if (now_ms - wrm_last_progress_ms_ > 4000) {
          // Deadlocked. Do NOT proceed past the wait (downstream packets need
          // the token the guest never wrote - dereferencing it crashes). Just
          // abandon the pending ring to the write pointer so the CP idles;
          // the guest re-submits and, if its producer un-wedges, recovers.
          XELOGW(
              "PM4 WAIT_REG_MEM addr={:08X} ref={:08X}: CPU<->GPU standoff "
              "({} ms) - abandoning the pending ring.",
              poll_reg_addr, ref, now_ms - wrm_last_progress_ms_);
          wrm_last_progress_counter_ = cnt;
          wrm_last_progress_ms_ = now_ms;
          wrm_deadlock_abort_ = true;
          wait_reg_mem_parked_.store(false, std::memory_order_relaxed);
          if (graphics_system_) {
            graphics_system_->PublishGpuIdentifier();
          }
          return false;
        }
      }

      if (wait >= 0x100) {
        PrepareForWait();
        if (cvars::vsync) {
          // WAIT_REG_MEM's `wait` is a poll interval, not a total timeout - the
          // hardware re-checks each interval. Clamp it: stale IB content
          // decodes `wait` to garbage (seen: 0xFFFFFFFF -> hours), which would
          // wedge the CP in one Sleep past every deadlock check below.
          uint32_t wait_ms = wait / 0x100;
          xe::threading::Sleep(
              std::chrono::milliseconds(wait_ms > 4 ? 4 : wait_ms));
        } else {
          xe::threading::Sleep(std::chrono::microseconds(100));
        }
        ReturnFromWait();
      } else {
        // Sleep 100us per spin to yield CPU slice to guest producer threads
        // (GameThread, DPC thread, allocator) so they can publish the polled token.
        xe::threading::Sleep(std::chrono::microseconds(100));
      }

      if (!worker_running_) {
        if (wait_reg_mem_parked_.load(std::memory_order_relaxed)) {
          wait_reg_mem_parked_.store(false, std::memory_order_relaxed);
          if (graphics_system_) {
            graphics_system_->PublishGpuIdentifier();
          }
        }
        return false;
      }
    }
  } while (!matched);

  if (is_memory && wait_reg_mem_parked_.load(std::memory_order_relaxed)) {
    wait_reg_mem_parked_.store(false, std::memory_order_relaxed);
    if (graphics_system_) {
      graphics_system_->PublishGpuIdentifier();
    }
  }
  return true;
}
XE_NOINLINE
bool COMMAND_PROCESSOR::ExecutePacketType3_REG_RMW(uint32_t packet,
                                                   uint32_t count) XE_RESTRICT {
  // register read/modify/write
  // ? (used during shader upload and edram setup)
  uint32_t rmw_info = reader_.ReadAndSwap<uint32_t>();
  uint32_t and_mask = reader_.ReadAndSwap<uint32_t>();
  uint32_t or_mask = reader_.ReadAndSwap<uint32_t>();
  uint32_t value = register_file_->values[rmw_info & 0x1FFF];
  if ((rmw_info >> 31) & 0x1) {
    // & reg
    value &= register_file_->values[and_mask & 0x1FFF];
  } else {
    // & imm
    value &= and_mask;
  }
  if ((rmw_info >> 30) & 0x1) {
    // | reg
    value |= register_file_->values[or_mask & 0x1FFF];
  } else {
    // | imm
    value |= or_mask;
  }
  COMMAND_PROCESSOR::WriteRegister(rmw_info & 0x1FFF, value);
  return true;
}

bool COMMAND_PROCESSOR::ExecutePacketType3_REG_TO_MEM(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  // Copy Register to Memory (?)
  // Count is 2, assuming a Register Addr and a Memory Addr.

  uint32_t reg_addr = reader_.ReadAndSwap<uint32_t>();
  uint32_t mem_addr = reader_.ReadAndSwap<uint32_t>();

  uint32_t reg_val;

  assert_true(reg_addr < RegisterFile::kRegisterCount);
  reg_val = register_file_->values[reg_addr];

  auto endianness = static_cast<xenos::Endian>(mem_addr & 0x3);
  mem_addr &= ~0x3;
  reg_val = GpuSwap(reg_val, endianness);
  xe::store(memory_->TranslatePhysical(mem_addr), reg_val);
  trace_writer_.WriteMemoryWrite(CpuToGpu(mem_addr), 4);

  static const bool log_wrm = std::getenv("XE_LOG_WRM") != nullptr;
  if (log_wrm && mem_addr >= 0x1FC00000 && mem_addr < 0x1FE00000) {
    XELOGW("WRM-TRACE REG_TO_MEM reg={:04X} -> mem={:08X} val={:08X}", reg_addr,
           mem_addr, reg_val);
  }
  return true;
}
XE_NOINLINE
bool COMMAND_PROCESSOR::ExecutePacketType3_MEM_WRITE(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  uint32_t write_addr = reader_.ReadAndSwap<uint32_t>();
  for (uint32_t i = 0; i < count - 1; i++) {
    uint32_t write_data = reader_.ReadAndSwap<uint32_t>();

    auto endianness = static_cast<xenos::Endian>(write_addr & 0x3);
    auto addr = write_addr & ~0x3;
    write_data = GpuSwap(write_data, endianness);
    xe::store(memory_->TranslatePhysical(addr), write_data);
    trace_writer_.WriteMemoryWrite(CpuToGpu(addr), 4);
    static const bool log_wrm = std::getenv("XE_LOG_WRM") != nullptr;
    if (log_wrm && addr >= 0x1FC00000 && addr < 0x1FE00000) {
      XELOGW("WRM-TRACE MEM_WRITE mem={:08X} val={:08X}", addr, write_data);
    }
    write_addr += 4;
  }

  return true;
}
XE_NOINLINE
bool COMMAND_PROCESSOR::ExecutePacketType3_COND_WRITE(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  // conditional write to memory or register
  uint32_t wait_info = reader_.ReadAndSwap<uint32_t>();
  uint32_t poll_reg_addr = reader_.ReadAndSwap<uint32_t>();
  uint32_t ref = reader_.ReadAndSwap<uint32_t>();
  uint32_t mask = reader_.ReadAndSwap<uint32_t>();
  uint32_t write_reg_addr = reader_.ReadAndSwap<uint32_t>();
  uint32_t write_data = reader_.ReadAndSwap<uint32_t>();
  uint32_t value;
  if (wait_info & 0x10) {
    // Memory.
    auto endianness = static_cast<xenos::Endian>(poll_reg_addr & 0x3);
    poll_reg_addr &= ~0x3;
    trace_writer_.WriteMemoryRead(CpuToGpu(poll_reg_addr), 4);
    value = xe::load<uint32_t>(memory_->TranslatePhysical(poll_reg_addr));
    value = GpuSwap(value, endianness);
  } else {
    // Register.
    assert_true(poll_reg_addr < RegisterFile::kRegisterCount);
    value = register_file_->values[poll_reg_addr];
  }
  bool matched = MatchValueAndRef(value & mask, ref, wait_info);

  if (matched) {
    // Write.
    if (wait_info & 0x100) {
      // Memory.
      auto endianness = static_cast<xenos::Endian>(write_reg_addr & 0x3);
      write_reg_addr &= ~0x3;
      write_data = GpuSwap(write_data, endianness);
      xe::store(memory_->TranslatePhysical(write_reg_addr), write_data);
      trace_writer_.WriteMemoryWrite(CpuToGpu(write_reg_addr), 4);
    } else {
      // Register.
      COMMAND_PROCESSOR::WriteRegister(write_reg_addr, write_data);
    }
  }
  return true;
}
XE_FORCEINLINE
void COMMAND_PROCESSOR::WriteEventInitiator(uint32_t value) XE_RESTRICT {
  register_file_->values[XE_GPU_REG_VGT_EVENT_INITIATOR] = value;
}
static const char* GetEventTypeName(uint32_t event_type) {
  switch (event_type) {
    case xenos::Event::VS_DEALLOC: return "VS_DEALLOC(0)";
    case xenos::Event::PS_DEALLOC: return "PS_DEALLOC(1)";
    case xenos::Event::VS_DONE_TS: return "VS_DONE_TS(2)";
    case xenos::Event::PS_DONE_TS: return "PS_DONE_TS(3)";
    case xenos::Event::CACHE_FLUSH_TS: return "CACHE_FLUSH_TS(4)";
    case xenos::Event::CONTEXT_DONE: return "CONTEXT_DONE(5)";
    case xenos::Event::CACHE_FLUSH: return "CACHE_FLUSH(6)";
    case xenos::Event::VIZQUERY_START: return "VIZQUERY_START(7)";
    case xenos::Event::VIZQUERY_END: return "VIZQUERY_END(8)";
    case xenos::Event::SC_WAIT_WC: return "SC_WAIT_WC(9)";
    case xenos::Event::MPASS_PS_CP_REFETCH: return "MPASS_PS_CP_REFETCH(10)";
    case xenos::Event::MPASS_PS_RST_START: return "MPASS_PS_RST_START(11)";
    case xenos::Event::MPASS_PS_INCR_START: return "MPASS_PS_INCR_START(12)";
    case xenos::Event::RST_PIX_CNT: return "RST_PIX_CNT(13)";
    case xenos::Event::RST_VTX_CNT: return "RST_VTX_CNT(14)";
    case xenos::Event::TILE_FLUSH: return "TILE_FLUSH(15)";
    case xenos::Event::CACHE_FLUSH_AND_INV_TS_EVENT: return "CACHE_FLUSH_AND_INV_TS_EVENT(20)";
    case xenos::Event::ZPASS_DONE: return "ZPASS_DONE(21)";
    case xenos::Event::CACHE_FLUSH_AND_INV_EVENT: return "CACHE_FLUSH_AND_INV_EVENT(22)";
    case xenos::Event::PERFCOUNTER_START: return "PERFCOUNTER_START(23)";
    case xenos::Event::PERFCOUNTER_STOP: return "PERFCOUNTER_STOP(24)";
    case xenos::Event::SCREEN_EXT_INIT: return "SCREEN_EXT_INIT(25)";
    case xenos::Event::SCREEN_EXT_RPT: return "SCREEN_EXT_RPT(26)";
    case xenos::Event::VS_FETCH_DONE_TS: return "VS_FETCH_DONE_TS(27)";
    default: return "UNKNOWN";
  }
}
bool COMMAND_PROCESSOR::ExecutePacketType3_EVENT_WRITE(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  // generate an event that creates a write to memory when completed
  uint32_t initiator = reader_.ReadAndSwap<uint32_t>();
  uint32_t event_type = initiator & 0x3f;
  // Writeback initiator.
  COMMAND_PROCESSOR::WriteEventInitiator(event_type);
  static const bool log_wrm = std::getenv("XE_LOG_WRM") != nullptr;
  if (count == 1) {
    // Just an event flag? Where does this write?
    if (log_wrm) {
      XELOGW("WRM-TRACE EVENT_WRITE (flag-only) initiator={:08X} type={} ({})",
             initiator, event_type, GetEventTypeName(event_type));
    }
  } else {
    // Timestamp / fence event: writes a value to a guest address when the CP
    // reaches this packet (CACHE_FLUSH_AND_INV_TS etc.). Stock Xenia asserts
    // and drops this - which is exactly the missing CP->CPU mailbox writeback
    // Fable II's GPU pump polls (0x1FC800xx WAIT_REG_MEM standoff). Read the
    // address + data and log; the actual write is done below.
    uint32_t address = reader_.ReadAndSwap<uint32_t>();
    uint32_t data_lo = 0;
    bool have_data = count >= 3;
    if (have_data) {
      data_lo = reader_.ReadAndSwap<uint32_t>();
    }
    if (count >= 4) {
      reader_.AdvanceRead((count - 3) * sizeof(uint32_t));  // data hi + extra
    }
    auto endianness = static_cast<xenos::Endian>(address & 0x3);
    uint32_t aligned = address & ~uint32_t(0x3);
    if (have_data) {
      uint32_t swapped = xenos::GpuSwap(data_lo, endianness);
      xe::store(memory_->TranslatePhysical(aligned), swapped);
      trace_writer_.WriteMemoryWrite(CpuToGpu(aligned), 4);
    }
    if (log_wrm) {
      XELOGW(
          "WRM-TRACE EVENT_WRITE initiator={:08X} type={} ({}) count={} -> addr={:08X} "
          "data={:08X} have_data={} (endian={})",
          initiator, event_type, GetEventTypeName(event_type), count, address, data_lo,
          have_data, static_cast<int>(endianness));
    }
  }

  // macos-arm64 Fable II bring-up, Phase 28: promoted to default. Real
  // hardware's EVENT_WRITE writes memory AND interrupts the CPU so it
  // doesn't have to poll; this only ever did the write half. Firing on
  // *every* EVENT_WRITE broke boot (routine events like CACHE_FLUSH fire
  // tens of thousands of times/run). CACHE_FLUSH_TS specifically fires
  // exactly once in every captured trace this session and, gated to just
  // that type, fixes the resource-completion livelock (verified live via
  // lldb: consumer/producer indices were equal - ring fully drained,
  // waiting on a write that never came) with no observed regression.
  // XE_EVENT_WRITE_INTERRUPT overrides for diagnostics: "all"/"1"/"true"
  // fires on every event type (known to break boot, kept for
  // comparison), "0"/"none"/"false" disables entirely, a bare number
  // fires only for that specific event type instead of CACHE_FLUSH_TS.
  bool should_fire_interrupt = (event_type == xenos::Event::CACHE_FLUSH_TS);
  if (const char* interrupt_env = std::getenv("XE_EVENT_WRITE_INTERRUPT")) {
    if (std::strcmp(interrupt_env, "all") == 0 ||
        std::strcmp(interrupt_env, "1") == 0 ||
        std::strcmp(interrupt_env, "true") == 0) {
      should_fire_interrupt = true;
    } else if (std::strcmp(interrupt_env, "0") == 0 ||
               std::strcmp(interrupt_env, "none") == 0 ||
               std::strcmp(interrupt_env, "false") == 0) {
      should_fire_interrupt = false;
    } else {
      char* endptr = nullptr;
      long val = std::strtol(interrupt_env, &endptr, 0);
      should_fire_interrupt =
          (endptr != interrupt_env && val == static_cast<long>(event_type));
    }
  }
  if (should_fire_interrupt && graphics_system_) {
    if (log_wrm) {
      XELOGW("WRM-TRACE EVENT_WRITE firing interrupt for type={} ({})",
             event_type, GetEventTypeName(event_type));
    }
    graphics_system_->DispatchInterruptCallback(1, 2);
  }
  return true;
}
XE_NOINLINE
bool COMMAND_PROCESSOR::ExecutePacketType3_EVENT_WRITE_SHD(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  // generate a VS|PS_done event
  uint32_t initiator = reader_.ReadAndSwap<uint32_t>();
  uint32_t address = reader_.ReadAndSwap<uint32_t>();
  uint32_t value = reader_.ReadAndSwap<uint32_t>();
  uint32_t event_type = initiator & 0x3F;
  // Writeback initiator.
  COMMAND_PROCESSOR::WriteEventInitiator(event_type);
  uint32_t data_value;
  if ((initiator >> 31) & 0x1) {
    // Write counter (GPU vblank counter?).
    data_value = counter_;
  } else {
    // Write value.
    data_value = value;
  }
  auto endianness = static_cast<xenos::Endian>(address & 0x3);
  uint32_t aligned_address = address & ~0x3;
  uint32_t swapped_data = GpuSwap(data_value, endianness);
  uint8_t* write_destination = memory_->TranslatePhysical(aligned_address);
  if (aligned_address > 0x1FFFFFFF) {
    uint32_t writeback_base =
        register_file_->values[XE_GPU_REG_WRITEBACK_START];
    uint32_t writeback_size = register_file_->values[XE_GPU_REG_WRITEBACK_SIZE];
    uint32_t writeback_offset = aligned_address - writeback_base;
    // check whether the guest has written writeback base. if they haven't, skip
    // the offset check
    if (writeback_base != 0 && writeback_offset < writeback_size) {
      write_destination =
          memory_->TranslateVirtual(0x7F000000 + writeback_offset);
    }
  }
  xe::store(write_destination, swapped_data);
  trace_writer_.WriteMemoryWrite(CpuToGpu(aligned_address), 4);

  static const bool log_wrm = std::getenv("XE_LOG_WRM") != nullptr;
  if (log_wrm) {
    XELOGW(
        "WRM-TRACE EVENT_WRITE_SHD initiator={:08X} type={} ({}) count={} -> addr={:08X} "
        "data={:08X} (endian={})",
        initiator, event_type, GetEventTypeName(event_type), count, address, data_value,
        static_cast<int>(endianness));
  }



  // Phase 28: promoted to default - see the matching comment in
  // ExecutePacketType3_EVENT_WRITE above.
  bool should_fire_interrupt = (event_type == xenos::Event::CACHE_FLUSH_TS);
  if (const char* interrupt_env = std::getenv("XE_EVENT_WRITE_INTERRUPT")) {
    if (std::strcmp(interrupt_env, "all") == 0 ||
        std::strcmp(interrupt_env, "1") == 0 ||
        std::strcmp(interrupt_env, "true") == 0) {
      should_fire_interrupt = true;
    } else if (std::strcmp(interrupt_env, "0") == 0 ||
               std::strcmp(interrupt_env, "none") == 0 ||
               std::strcmp(interrupt_env, "false") == 0) {
      should_fire_interrupt = false;
    } else {
      char* endptr = nullptr;
      long val = std::strtol(interrupt_env, &endptr, 0);
      should_fire_interrupt =
          (endptr != interrupt_env && val == static_cast<long>(event_type));
    }
  }
  if (should_fire_interrupt && graphics_system_) {
    if (log_wrm) {
      XELOGW("WRM-TRACE EVENT_WRITE_SHD firing interrupt for type={} ({})",
             event_type, GetEventTypeName(event_type));
    }
    graphics_system_->DispatchInterruptCallback(1, 2);
  }
  return true;
}

bool COMMAND_PROCESSOR::ExecutePacketType3_EVENT_WRITE_EXT(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  // generate a screen extent event
  uint32_t initiator = reader_.ReadAndSwap<uint32_t>();
  uint32_t address = reader_.ReadAndSwap<uint32_t>();
  uint32_t event_type = initiator & 0x3F;
  // Writeback initiator.
  COMMAND_PROCESSOR::WriteEventInitiator(event_type);
  auto endianness = static_cast<xenos::Endian>(address & 0x3);
  address &= ~0x3;

  // Let us hope we can fake this.
  // This callback tells the driver the xy coordinates affected by a previous
  // drawcall.
  // https://www.google.com/patents/US20060055701
  uint16_t extents[] = {
      byte_swap<unsigned short>(0 >> 3),  // min x
      byte_swap<unsigned short>(xenos::kTexture2DCubeMaxWidthHeight >>
                                3),       // max x
      byte_swap<unsigned short>(0 >> 3),  // min y
      byte_swap<unsigned short>(xenos::kTexture2DCubeMaxWidthHeight >>
                                3),  // max y
      byte_swap<unsigned short>(0),  // min z
      byte_swap<unsigned short>(1),  // max z
  };
  assert_true(endianness == xenos::Endian::k8in16);

  uint16_t* destination = (uint16_t*)memory_->TranslatePhysical(address);

  for (unsigned i = 0; i < 6; ++i) {
    destination[i] = extents[i];
  }

  trace_writer_.WriteMemoryWrite(CpuToGpu(address), sizeof(extents));
  static const bool log_wrm = std::getenv("XE_LOG_WRM") != nullptr;
  if (log_wrm) {
    XELOGW("WRM-TRACE PM4_EVENT_WRITE_EXT: initiator={:08X} type={} ({}) addr={:08X} endian={}",
           initiator, event_type, GetEventTypeName(event_type), address, static_cast<int>(endianness));
  }
  return true;
}

XE_NOINLINE
bool COMMAND_PROCESSOR::ExecutePacketType3_EVENT_WRITE_ZPD(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  assert_true(count == 1);
  uint32_t initiator = reader_.ReadAndSwap<uint32_t>();
  uint32_t event_type = initiator & 0x3F;
  // Writeback initiator.
  COMMAND_PROCESSOR::WriteEventInitiator(event_type);
  static const bool log_wrm = std::getenv("XE_LOG_WRM") != nullptr;
  if (log_wrm) {
    XELOGW("WRM-TRACE PM4_EVENT_WRITE_ZPD: initiator={:08X} type={} ({})",
           initiator, event_type, GetEventTypeName(event_type));
  }

  uint32_t report_address =
      register_file_->values[XE_GPU_REG_RB_SAMPLE_COUNT_ADDR];
  uint32_t report_record_base = XenosZPDReport::GetRecordBase(report_address);
  bool is_begin_record = XenosZPDReport::IsBeginRecord(report_address);
  bool is_end_record = XenosZPDReport::IsEndRecord(report_address);

  xe_gpu_depth_sample_counts* report =
      report_record_base
          ? memory_->TranslatePhysical<xe_gpu_depth_sample_counts*>(
                report_record_base)
          : nullptr;

  // True if the record has the pending D3D sentinel.
  // Useful as a hint, but not authoritative for report boundaries.
  // QueryBatch titles can have multiple pending sentinels in a row and don't
  // necessarily update in an order we currently observe.
  bool guest_marks_end = report && XenosZPDReport::HasPendingSentinel(report);
  bool logical_active = zpd_active_segment_.logical_active;

  // QueryBatch fake fallback, which ignores record layout and just returns an
  // incrementing sample count on each event.
  if (cvars::occlusion_query_querybatch_range > 0) {
    uint32_t sample_count =
        XenosZPDReport::QueryBatchFakeSamples(querybatch_zpd_sample_count_);
    if (report) {
      // Both QueryBatch and conventional fake samples skip elective saturation.
      XenosZPDReport::WriteSampleCount(report, sample_count, false);
    }
    return true;
  }

  if (GetZPDMode() != ZPDMode::kFake && !zpd_force_fake_fallback_) {
    if (logical_active && is_end_record) {
      COMMAND_PROCESSOR::EndZPDReport(report_address, false);
      return true;
    }
    if (is_begin_record) {
      // Clear the record so the game knows the BEGIN was processed and
      // stale sentinel data from a prior query lifetime doesn't persist.
      if (report) {
        std::memset(report, 0, sizeof(xe_gpu_depth_sample_counts));
      }
      COMMAND_PROCESSOR::BeginZPDReport(report_address);
      return true;
    }
    if (!logical_active && is_end_record) {
      // No logical report is active for this slot, so this is likely an
      // orphaned END. In fast mode, replay the last cached delta so polling
      // code does not sit on the sentinel forever.
      if (GetZPDMode() == ZPDMode::kFast || GetZPDMode() == ZPDMode::kFastAlt) {
        uint32_t cached_delta = 1;
        auto cache_it = fast_zpd_report_cached_values_.find(report_record_base);
        if (cache_it != fast_zpd_report_cached_values_.end()) {
          cached_delta = cache_it->second;
        }
        COMMAND_PROCESSOR::WriteZPDReport(0, report_record_base, 0,
                                          cached_delta, false);
      } else {
        // In strict mode, just pump in case a previous report has resolved.
        COMMAND_PROCESSOR::PumpQueryResolves();
      }
      return true;
    }
    // Address is neither BEGIN nor END (non-standard layout). Fall through
    // to the fake path so the guest at least gets a result written rather
    // than leaving the sentinel in place forever.
  }

  // Conventional fake fallback, which only touches records marked as pending.
  if (cvars::occlusion_query_fake_lower_threshold < 0 || !report_record_base ||
      !guest_marks_end) {
    return true;
  }

  fake_zpd_sample_count_ =
      (fake_zpd_sample_count_ <=
       static_cast<uint32_t>(cvars::occlusion_query_fake_lower_threshold))
          ? static_cast<uint32_t>(cvars::occlusion_query_fake_upper_threshold)
          : fake_zpd_sample_count_ - 1;

  XenosZPDReport::WriteSampleCount(report, fake_zpd_sample_count_, false);
  return true;
}

bool COMMAND_PROCESSOR::ExecutePacketType3Draw(
    uint32_t packet, const char* opcode_name, uint32_t viz_query_condition,
    uint32_t count_remaining) XE_RESTRICT {
  // if viz_query_condition != 0, this is a conditional draw based on viz query.
  // This ID matches the one issued in PM4_VIZ_QUERY
  // uint32_t viz_id = viz_query_condition & 0x3F;
  // when true, render conditionally based on query result
  // uint32_t viz_use = viz_query_condition & 0x100;

  assert_not_zero(count_remaining);
  if (!count_remaining) {
    XELOGE("{}: Packet too small, can't read VGT_DRAW_INITIATOR", opcode_name);
    return false;
  }
  reg::VGT_DRAW_INITIATOR vgt_draw_initiator;
  vgt_draw_initiator.value = reader_.ReadAndSwap<uint32_t>();
  --count_remaining;

  register_file_->values[XE_GPU_REG_VGT_DRAW_INITIATOR] =
      vgt_draw_initiator.value;
  bool draw_succeeded = true;
  // TODO(Triang3l): Remove IndexBufferInfo and replace handling of all this
  // with PrimitiveProcessor when the old Vulkan renderer is removed.
  bool is_indexed = false;
  IndexBufferInfo index_buffer_info;
  switch (vgt_draw_initiator.source_select) {
    case xenos::SourceSelect::kDMA: {
      // Indexed draw.
      is_indexed = true;

      // Two separate bounds checks so if there's only one missing register
      // value out of two, one uint32_t will be skipped in the command buffer,
      // not two.
      assert_not_zero(count_remaining);
      if (!count_remaining) {
        XELOGE("{}: Packet too small, can't read VGT_DMA_BASE", opcode_name);
        return false;
      }
      // Wrap writeback & XPS addresses to their physical backing.
      uint32_t vgt_dma_base = CpuToGpu(reader_.ReadAndSwap<uint32_t>());
      --count_remaining;
      register_file_->values[XE_GPU_REG_VGT_DMA_BASE] = vgt_dma_base;
      reg::VGT_DMA_SIZE vgt_dma_size;
      assert_not_zero(count_remaining);
      if (!count_remaining) {
        XELOGE("{}: Packet too small, can't read VGT_DMA_SIZE", opcode_name);
        return false;
      }
      vgt_dma_size.value = reader_.ReadAndSwap<uint32_t>();
      --count_remaining;
      register_file_->values[XE_GPU_REG_VGT_DMA_SIZE] = vgt_dma_size.value;

      uint32_t index_size_bytes =
          vgt_draw_initiator.index_size == xenos::IndexFormat::kInt16
              ? sizeof(uint16_t)
              : sizeof(uint32_t);
      // The base address must already be word-aligned according to the R6xx
      // documentation, but for safety.
      index_buffer_info.guest_base = vgt_dma_base & ~(index_size_bytes - 1);
      index_buffer_info.endianness = vgt_dma_size.swap_mode;
      index_buffer_info.format = vgt_draw_initiator.index_size;
      index_buffer_info.length = vgt_dma_size.num_words * index_size_bytes;
      index_buffer_info.count = vgt_draw_initiator.num_indices;
    } break;
    case xenos::SourceSelect::kImmediate: {
      // TODO(Triang3l): VGT_IMMED_DATA.
      XELOGE(
          "{}: Using immediate vertex indices, which are not supported yet. "
          "Report the game to Xenia developers!",
          opcode_name, uint32_t(vgt_draw_initiator.source_select));
      draw_succeeded = false;
      assert_always();
    } break;
    case xenos::SourceSelect::kAutoIndex: {
      // Auto draw.
      index_buffer_info.guest_base = 0;
      index_buffer_info.length = 0;
    } break;
    default: {
      // Invalid source selection.
      draw_succeeded = false;
      assert_unhandled_case(vgt_draw_initiator.source_select);
    } break;
  }

  // Skip to the next command, for example, if there are immediate indexes that
  // we don't support yet.
  reader_.AdvanceRead(count_remaining * sizeof(uint32_t));

  if (draw_succeeded) {
    auto viz_query = register_file_->Get<reg::PA_SC_VIZ_QUERY>();
    if (!(viz_query.viz_query_ena && viz_query.kill_pix_post_hi_z)) {
      // macos-arm64 Fable II bring-up, Phase 16: trace every draw's active
      // vertex/pixel shader hashes + primitive info, to identify which draw
      // is the Bink-video/logo quad by its shape (small index count, likely
      // a full-screen or near-full-screen 4-vertex quad) rather than by
      // guessing from PM4_IM_LOAD_IMMEDIATE timing, which Phase 15's visual
      // A/B test showed doesn't actually explain the warped-text artifact.
      static const bool log_draw = std::getenv("XE_LOG_DRAW") != nullptr;
      if (log_draw) {
        XELOGW(
            "DRAW-TRACE prim_type={} num_indices={} indexed={} vs_hash={:016X}"
            " ps_hash={:016X}",
            uint32_t(vgt_draw_initiator.prim_type),
            vgt_draw_initiator.num_indices, is_indexed,
            active_vertex_shader_ ? active_vertex_shader_->ucode_data_hash()
                                   : 0,
            active_pixel_shader_ ? active_pixel_shader_->ucode_data_hash()
                                 : 0);
        // Phase 16 continued: for the specific rect-list (prim_type=8,
        // num_indices=3) full-screen blit shape identified as the
        // video/logo-compositing draw, dump every texture fetch constant the
        // active pixel shader actually samples from - format/dimensions/
        // address/tiled/endianness - to check whether the decoded Bink frame
        // is already wrong (bad format, wrong dims, garbage address) before
        // it ever reaches the shader, since no vertex/pixel shader involved
        // in this draw has ever thrown a translation error.
        if (vgt_draw_initiator.prim_type == xenos::PrimitiveType::kRectangleList &&
            active_pixel_shader_) {
          for (const auto& binding : active_pixel_shader_->texture_bindings()) {
            xenos::xe_gpu_texture_fetch_t fetch =
                register_file_->GetTextureFetch(binding.fetch_constant);
            XELOGW(
                "  TEX-FETCH slot={} type={} format={} dim={} w={} h={} "
                "base_addr={:08X} pitch={} tiled={} endian={} raw=[{:08X} "
                "{:08X} {:08X} {:08X} {:08X} {:08X}]",
                binding.fetch_constant, uint32_t(fetch.type),
                uint32_t(fetch.format), uint32_t(fetch.dimension),
                fetch.size_2d.width + 1, fetch.size_2d.height + 1,
                fetch.base_address << 12, fetch.pitch, fetch.tiled,
                uint32_t(fetch.endianness), fetch.dword_0, fetch.dword_1,
                fetch.dword_2, fetch.dword_3, fetch.dword_4, fetch.dword_5);
          }
        }
      }
      Pm4DumpShaderIfTargeted(active_vertex_shader_, "vertex");
      Pm4DumpShaderIfTargeted(active_pixel_shader_, "pixel");
      // TODO(Triang3l): Don't drop the draw call completely if the vertex
      // shader has memexport.
      // TODO(Triang3l || JoelLinn): Handle this properly in the render
      // backends.
      draw_succeeded = COMMAND_PROCESSOR::IssueDraw(
          vgt_draw_initiator.prim_type, vgt_draw_initiator.num_indices,
          is_indexed ? &index_buffer_info : nullptr,
          xenos::IsMajorModeExplicit(vgt_draw_initiator.major_mode,
                                     vgt_draw_initiator.prim_type));
      if (!draw_succeeded) {
        XELOGE("{}({}, {}, {}): Failed in backend", opcode_name,
               vgt_draw_initiator.num_indices,
               uint32_t(vgt_draw_initiator.prim_type),
               uint32_t(vgt_draw_initiator.source_select));
      }
    }
  }

  // If read the packed correctly, but merely couldn't execute it (because of,
  // for instance, features not supported by the host), don't terminate command
  // buffer processing as that would leave rendering in a way more inconsistent
  // state than just a single dropped draw command.
  return true;
}

bool COMMAND_PROCESSOR::ExecutePacketType3_DRAW_INDX(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  // "initiate fetch of index buffer and draw"
  // Generally used by Xbox 360 Direct3D 9 for kDMA and kAutoIndex sources.
  // With a viz query token as the first one.
  uint32_t count_remaining = count;
  assert_not_zero(count_remaining);
  if (!count_remaining) {
    XELOGE("PM4_DRAW_INDX: Packet too small, can't read the viz query token");
    return false;
  }
  uint32_t viz_query_condition = reader_.ReadAndSwap<uint32_t>();
  --count_remaining;
  return COMMAND_PROCESSOR::ExecutePacketType3Draw(
      packet, "PM4_DRAW_INDX", viz_query_condition, count_remaining);
}

bool COMMAND_PROCESSOR::ExecutePacketType3_DRAW_INDX_2(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  // "draw using supplied indices in packet"
  // Generally used by Xbox 360 Direct3D 9 for kAutoIndex source.
  // No viz query token.
  return COMMAND_PROCESSOR::ExecutePacketType3Draw(packet, "PM4_DRAW_INDX_2", 0,
                                                   count);
}
XE_FORCEINLINE
bool COMMAND_PROCESSOR::ExecutePacketType3_SET_CONSTANT(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  // load constant into chip and to memory
  // PM4_REG(reg) ((0x4 << 16) | (GSL_HAL_SUBBLOCK_OFFSET(reg)))
  //                                     reg - 0x2000
  uint32_t offset_type = reader_.ReadAndSwap<uint32_t>();
  uint32_t index = offset_type & 0x7FF;
  uint32_t type = (offset_type >> 16) & 0xFF;
  uint32_t countm1 = count - 1;
  switch (type) {
    case 0:  // ALU
      // index += 0x4000;
      // COMMAND_PROCESSOR::WriteRegisterRangeFromRing( index, countm1);
      COMMAND_PROCESSOR::WriteALURangeFromRing(&reader_, index, countm1);
      break;
    case 1:  // FETCH
      // macos-arm64 Fable II bring-up, Phase 19: identify which PM4
      // mechanism is responsible for writes to fetch-constant slot 0's
      // dwords (0-5) - SET_CONSTANT(FETCH), SET_CONSTANT2 ("INCR_UPDATE_
      // STATE" per xenos.h - possibly a different state-update timing
      // semantic xenia doesn't model), or LOAD_ALU_CONSTANT(FETCH), to
      // narrow down the constant-resolution-timing hypothesis from Phase
      // 17/18.
      if (std::getenv("XE_LOG_FETCH0") && index <= 5 &&
          index + countm1 >= 0) {
        XELOGW("FETCH0-SRC via SET_CONSTANT(FETCH) index={} countm1={}",
               index, countm1);
        Pm4DumpRecent("SET_CONSTANT(FETCH) touching slot 0");
      }
      COMMAND_PROCESSOR::WriteFetchRangeFromRing(&reader_, index, countm1);

      break;
    case 2:  // BOOL
      COMMAND_PROCESSOR::WriteBoolRangeFromRing(&reader_, index, countm1);

      break;
    case 3:  // LOOP

      COMMAND_PROCESSOR::WriteLoopRangeFromRing(&reader_, index, countm1);

      break;
    case 4:  // REGISTERS

      COMMAND_PROCESSOR::WriteREGISTERSRangeFromRing(&reader_, index, countm1);

      break;
    default:
      assert_always();
      reader_.AdvanceRead((count - 1) * sizeof(uint32_t));
      return true;
  }

  return true;
}
XE_NOINLINE
bool COMMAND_PROCESSOR::ExecutePacketType3_SET_CONSTANT2(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  uint32_t offset_type = reader_.ReadAndSwap<uint32_t>();
  uint32_t index = offset_type & 0xFFFF;
  uint32_t countm1 = count - 1;

  // Phase 19: same check, for the raw-register "INCR_UPDATE_STATE" path -
  // this writes ANY register directly (not typed ALU/FETCH/BOOL/LOOP), so
  // check against the absolute fetch-constant-slot-0 register range
  // (0x4800-0x4805) rather than a FETCH-relative index.
  if (std::getenv("XE_LOG_FETCH0") && index <= 0x4805 &&
      index + countm1 >= 0x4800) {
    XELOGW("FETCH0-SRC via SET_CONSTANT2(INCR_UPDATE_STATE) index={:04X} "
           "countm1={}",
           index, countm1);
    Pm4DumpRecent("SET_CONSTANT2 touching slot 0");
  }

  COMMAND_PROCESSOR::WriteRegisterRangeFromRing(&reader_, index, countm1);

  return true;
}
XE_FORCEINLINE
bool COMMAND_PROCESSOR::ExecutePacketType3_LOAD_ALU_CONSTANT(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  // load constants from memory
  uint32_t address = reader_.ReadAndSwap<uint32_t>();
  address &= 0x3FFFFFFF;
  uint32_t offset_type = reader_.ReadAndSwap<uint32_t>();
  uint32_t index = offset_type & 0x7FF;
  uint32_t size_dwords = reader_.ReadAndSwap<uint32_t>();
  size_dwords &= 0xFFF;
  uint32_t type = (offset_type >> 16) & 0xFF;

  auto xlat_address = (uint32_t*)memory_->TranslatePhysical(address);

  switch (type) {
    case 0:  // ALU
      trace_writer_.WriteMemoryRead(CpuToGpu(address), size_dwords * 4);
      COMMAND_PROCESSOR::WriteALURangeFromMem(index, xlat_address, size_dwords);

      break;
    case 1:  // FETCH
      if (std::getenv("XE_LOG_FETCH0") && index <= 5 &&
          index + size_dwords >= 0) {
        XELOGW("FETCH0-SRC via LOAD_ALU_CONSTANT(FETCH) index={} "
               "size_dwords={} addr={:08X}",
               index, size_dwords, address);
        Pm4DumpRecent("LOAD_ALU_CONSTANT(FETCH) touching slot 0");
      }
      trace_writer_.WriteMemoryRead(CpuToGpu(address), size_dwords * 4);
      COMMAND_PROCESSOR::WriteFetchRangeFromMem(index, xlat_address,
                                                size_dwords);
      break;
    case 2:  // BOOL
      trace_writer_.WriteMemoryRead(CpuToGpu(address), size_dwords * 4);

      COMMAND_PROCESSOR::WriteBoolRangeFromMem(index, xlat_address,
                                               size_dwords);
      break;
    case 3:  // LOOP
      trace_writer_.WriteMemoryRead(CpuToGpu(address), size_dwords * 4);

      COMMAND_PROCESSOR::WriteLoopRangeFromMem(index, xlat_address,
                                               size_dwords);

      break;
    case 4:  // REGISTERS
      // chrispy: todo, REGISTERS cannot write any special regs, so optimize for
      // that
      trace_writer_.WriteMemoryRead(CpuToGpu(address), size_dwords * 4);

      COMMAND_PROCESSOR::WriteREGISTERSRangeFromMem(index, xlat_address,
                                                    size_dwords);
      break;
    default:
      assert_always();
      return true;
  }

  return true;
}

bool COMMAND_PROCESSOR::ExecutePacketType3_SET_SHADER_CONSTANTS(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  uint32_t offset_type = reader_.ReadAndSwap<uint32_t>();
  uint32_t index = offset_type & 0xFFFF;
  uint32_t countm1 = count - 1;
  COMMAND_PROCESSOR::WriteRegisterRangeFromRing(&reader_, index, countm1);

  return true;
}

bool COMMAND_PROCESSOR::ExecutePacketType3_IM_LOAD(uint32_t packet,
                                                   uint32_t count) XE_RESTRICT {
  SCOPE_profile_cpu_f("gpu");

  // load sequencer instruction memory (pointer-based)
  uint32_t addr_type = reader_.ReadAndSwap<uint32_t>();
  auto shader_type = static_cast<xenos::ShaderType>(addr_type & 0x3);
  uint32_t addr = addr_type & ~0x3;
  uint32_t start_size = reader_.ReadAndSwap<uint32_t>();
  uint32_t start = start_size >> 16;
  uint32_t size_dwords = start_size & 0xFFFF;  // dwords
  assert_true(start == 0);
  trace_writer_.WriteMemoryRead(CpuToGpu(addr), size_dwords * 4);
  // macos-arm64 Fable II bring-up diagnostic: PHASE 9 - prove/disprove the CP
  // outrunning the guest's shader-microcode producer. IM_LOAD reads guest
  // memory the instant this packet executes; if the producer (another guest
  // thread, or the same thread a few stores earlier) hasn't finished writing
  // size_dwords worth of microcode yet, LoadShader hashes a partially-written
  // buffer - which is exactly what Phase 8 found (same header/size, different
  // hash every load). XE_IM_LOAD_DELAY_US (microseconds) stalls right here,
  // before the read, giving the producer a window to finish. Off by default;
  // this is a diagnostic, not a fix - a real fix needs the guest's actual
  // completion signal, not a fixed guess.
  if (uint32_t delay_us = []() -> uint32_t {
        static const uint32_t v = [] {
          const char* e = std::getenv("XE_IM_LOAD_DELAY_US");
          return e ? uint32_t(std::atoi(e)) : 0u;
        }();
        return v;
      }()) {
    xe::threading::Sleep(std::chrono::microseconds(delay_us));
  }
  auto shader = COMMAND_PROCESSOR::LoadShader(
      shader_type, addr, memory_->TranslatePhysical<uint32_t*>(addr),
      size_dwords);
  static const bool log_wrm_imload = std::getenv("XE_LOG_WRM") != nullptr;
  if (log_wrm_imload && shader_type == xenos::ShaderType::kVertex) {
    XELOGW("IM_LOAD-TRACE vertex addr={:08X} dwords={} hash={:016X}", addr,
           size_dwords, shader->ucode_data_hash());
  }
  switch (shader_type) {
    case xenos::ShaderType::kVertex:
      active_vertex_shader_ = shader;
      break;
    case xenos::ShaderType::kPixel:
      active_pixel_shader_ = shader;
      break;
    default:
      assert_unhandled_case(shader_type);
      return false;
  }
  return true;
}

bool COMMAND_PROCESSOR::ExecutePacketType3_IM_LOAD_IMMEDIATE(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  SCOPE_profile_cpu_f("gpu");

  // load sequencer instruction memory (code embedded in packet)
  uint32_t dword0 = reader_.ReadAndSwap<uint32_t>();
  uint32_t dword1 = reader_.ReadAndSwap<uint32_t>();
  auto shader_type = static_cast<xenos::ShaderType>(dword0);
  uint32_t start_size = dword1;
  uint32_t start = start_size >> 16;
  uint32_t size_dwords = start_size & 0xFFFF;  // dwords
  assert_true(start == 0);
  assert_true(reader_.read_count() >= size_dwords * 4);
  assert_true(count - 2 >= size_dwords);
  // macos-arm64 Fable II bring-up, Phase 10 correction: this shader is
  // embedded directly in the command stream, but that does NOT mean it can't
  // race the guest - "the IB dispatch was published" only proves the guest
  // finished writing the ring/IB *header*; it says nothing about whether the
  // guest CPU thread had actually finished stamping this specific packet's
  // payload bytes into that same buffer region before xenia's CP, running at
  // a different relative pace than real hardware, got there. Reuse
  // XE_IM_LOAD_DELAY_US here too (same knob as PM4_IM_LOAD) to test that:
  // trace evidence shows this exact 285-dword shader (E0544009... header)
  // hashes differently almost every load, with a different "SHADER-OP
  // unknown" opcode each time - the signature of a torn read, not a stable
  // unsupported instruction - and the prior Phase 9 delay experiment never
  // actually covered this path, since that shader is delivered via
  // IM_LOAD_IMMEDIATE, not IM_LOAD.
  if (uint32_t delay_us = []() -> uint32_t {
        static const uint32_t v = [] {
          const char* e = std::getenv("XE_IM_LOAD_DELAY_US");
          return e ? uint32_t(std::atoi(e)) : 0u;
        }();
        return v;
      }()) {
    xe::threading::Sleep(std::chrono::microseconds(delay_us));
  }
  // Log where in the buffer this load sits (ring_off/in_ib line up with the
  // PM4-STREAM dump's [P ]/[IB] entries) plus the hash, so a later
  // "SHADER-OP unknown ... hash=X" can be matched back to exactly which
  // buffer and offset it came from, and Pm4DumpRecent's rolling log shows
  // what was executed immediately before it.
  static const bool log_wrm_immediate = std::getenv("XE_LOG_WRM") != nullptr;
  uint32_t im_load_immediate_ring_off = uint32_t(reader_.read_offset());
  // Phase 11: snapshot the payload bytes (and the guest address they live
  // at) BEFORE LoadShader consumes them, so Pm4ImLoadImmDiagnose can diff
  // this observation against the last time this exact address was loaded,
  // and re-check the same bytes shortly after the load "completes" to catch
  // the guest still writing them. Gated to the known-corrupting shape
  // (vertex, 285 dwords) to bound overhead - opt-in via XE_IM_LOAD_LIFECYCLE.
  static const bool log_lifecycle =
      std::getenv("XE_IM_LOAD_LIFECYCLE") != nullptr;
  const bool lifecycle_this_load = log_lifecycle &&
                                    shader_type == xenos::ShaderType::kVertex &&
                                    size_dwords == 285;
  uint32_t lifecycle_guest_addr = 0;
  std::vector<uint32_t> lifecycle_snap0;
  const uint32_t* lifecycle_host_words = nullptr;
  if (lifecycle_this_load) {
    lifecycle_host_words = reinterpret_cast<const uint32_t*>(reader_.read_ptr());
    lifecycle_guest_addr = memory_->HostToGuestVirtual(
        reinterpret_cast<const void*>(reader_.read_ptr()));
    lifecycle_snap0.resize(size_dwords);
    for (uint32_t i = 0; i < size_dwords; ++i) {
      lifecycle_snap0[i] = xe::load_and_swap<uint32_t>(lifecycle_host_words + i);
    }
  }
  auto shader = COMMAND_PROCESSOR::LoadShader(
      shader_type, uint32_t(reader_.read_ptr()),
      reinterpret_cast<uint32_t*>(reader_.read_ptr()), size_dwords);
  if (log_wrm_immediate && shader_type == xenos::ShaderType::kVertex) {
    XELOGW(
        "IM_LOAD_IMMEDIATE-TRACE vertex in_ib={} ring_off={:06X} dwords={} "
        "hash={:016X}",
        g_pm4_in_ib, im_load_immediate_ring_off, size_dwords,
        shader->ucode_data_hash());
    Pm4DumpRecent("IM_LOAD_IMMEDIATE vertex shader load");
  }
  if (lifecycle_this_load) {
    Pm4ImLoadImmDiagnose(lifecycle_guest_addr, im_load_immediate_ring_off,
                          g_pm4_in_ib, size_dwords, lifecycle_snap0,
                          lifecycle_host_words, shader->ucode_data_hash());
  }
  switch (shader_type) {
    case xenos::ShaderType::kVertex:
      active_vertex_shader_ = shader;
      break;
    case xenos::ShaderType::kPixel:
      active_pixel_shader_ = shader;
      break;
    default:
      assert_unhandled_case(shader_type);
      return false;
  }
  reader_.AdvanceRead(size_dwords * sizeof(uint32_t));
  return true;
}

/*
        todo: shouldn't this do something?
*/

bool COMMAND_PROCESSOR::ExecutePacketType3_INVALIDATE_STATE(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  // selective invalidation of state pointers
  /*uint32_t mask =*/reader_.ReadAndSwap<uint32_t>();
  // driver_->InvalidateState(mask);
  return true;
}

bool COMMAND_PROCESSOR::ExecutePacketType3_VIZ_QUERY(
    uint32_t packet, uint32_t count) XE_RESTRICT {
  // begin/end initiator for viz query extent processing
  // https://www.google.com/patents/US20050195186
  assert_true(count == 1);

  uint32_t dword0 = reader_.ReadAndSwap<uint32_t>();

  uint32_t id = dword0 & 0x3F;
  uint32_t end = dword0 & 0x100;
  if (!end) {
    // begin a new viz query @ id
    // On hardware this clears the internal state of the scan converter (which
    // is different to the register)
    COMMAND_PROCESSOR::WriteEventInitiator(VIZQUERY_START);
    // XELOGGPU("Begin viz query ID {:02X}", id);
  } else {
    // end the viz query
    COMMAND_PROCESSOR::WriteEventInitiator(VIZQUERY_END);
    // XELOGGPU("End viz query ID {:02X}", id);
    // The scan converter writes the internal result back to the register here.
    // We just fake it and say it was visible in case it is read back.
    if (id < 32) {
      register_file_->values[XE_GPU_REG_PA_SC_VIZ_QUERY_STATUS_0] |= uint32_t(1)
                                                                     << id;
    } else {
      register_file_->values[XE_GPU_REG_PA_SC_VIZ_QUERY_STATUS_1] |=
          uint32_t(1) << (id - 32);
    }
  }

  return true;
}

uint32_t COMMAND_PROCESSOR::ExecutePrimaryBuffer(uint32_t read_index,
                                                 uint32_t write_index) {
  SCOPE_profile_cpu_f("gpu");
#if XE_ENABLE_TRACE_WRITER_INSTRUMENTATION == 1
  // If we have a pending trace stream open it now. That way we ensure we get
  // all commands.
  if (!trace_writer_.is_open() && trace_state_ == TraceState::kStreaming) {
    uint32_t title_id = kernel_state_->GetExecutableModule()
                            ? kernel_state_->GetExecutableModule()->title_id()
                            : 0;
    auto file_name = fmt::format("{:08X}_stream.xtr", title_id);
    auto path = trace_stream_path_ / file_name;
    trace_writer_.Open(path, title_id);
    InitializeTrace();
  }
#endif
  // Adjust pointer base.
  uint32_t start_ptr = primary_buffer_ptr_ + read_index * sizeof(uint32_t);
  start_ptr = (primary_buffer_ptr_ & ~0x1FFFFFFF) | (start_ptr & 0x1FFFFFFF);
  uint32_t end_ptr = primary_buffer_ptr_ + write_index * sizeof(uint32_t);
  end_ptr = (primary_buffer_ptr_ & ~0x1FFFFFFF) | (end_ptr & 0x1FFFFFFF);

  trace_writer_.WritePrimaryBufferStart(start_ptr, write_index - read_index);

  // Execute commands!

  RingBuffer old_reader = reader_;
  new (&reader_) RingBuffer(memory_->TranslatePhysical(primary_buffer_ptr_),
                            primary_buffer_size_);

  reader_.set_read_offset(read_index * sizeof(uint32_t));
  reader_.set_write_offset(write_index * sizeof(uint32_t));
  // prefetch the wraparound range
  // it likely is already in L3 cache, but in a zen system it may be another
  // chiplets l3
  reader_.BeginPrefetchedRead<swcache::PrefetchTag::Level2>(
      GetCurrentRingReadCount());
  do {
    if (!COMMAND_PROCESSOR::ExecutePacket()) {
      // macos-arm64 Fable II bring-up: no assert_always() (SIGTRAPs in this
      // NDEBUG build). Stale ring content reaches here; just stop.
      XELOGE("**** PRIMARY RINGBUFFER: Failed to execute packet.");
      Pm4DumpRecent("primary ring bad packet");
      break;
    }
    if (wrm_deadlock_abort_) {
      // A WAIT_REG_MEM downstream gave up on a CPU<->GPU standoff. Drop the
      // rest of the pending ring to the write pointer: the guest sees the GPU
      // drained, unblocks its pump loop, and re-submits.
      wrm_deadlock_abort_ = false;
      XELOGW(
          "PM4 primary buffer abandoned to the ring head after a WAIT_REG_MEM "
          "deadlock.");
      reader_.set_read_offset(write_index * sizeof(uint32_t));
      break;
    }
  } while (reader_.read_count());

  COMMAND_PROCESSOR::OnPrimaryBufferEnd();

  trace_writer_.WritePrimaryBufferEnd();

  reader_ = old_reader;
  return write_index;
}

void COMMAND_PROCESSOR::ExecutePacket(uint32_t ptr, uint32_t count) {
  // Execute commands!
  RingBuffer old_reader = reader_;

  new (&reader_)
      RingBuffer{memory_->TranslatePhysical(ptr), count * sizeof(uint32_t)};

  reader_.set_write_offset(count * sizeof(uint32_t));

  do {
    if (!COMMAND_PROCESSOR::ExecutePacket()) {
      XELOGE("**** ExecutePacket: Failed to execute packet.");
      break;
    }
    if (wrm_deadlock_abort_) {
      break;
    }
  } while (reader_.read_count());
  reader_ = old_reader;
}
