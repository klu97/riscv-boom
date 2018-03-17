package boom

import Chisel._
import freechips.rocketchip.config.{Parameters, Field}

case class Lab3Parameters(
  enabled: Boolean = true,
  history_length: Int = 9,
  info_size: Int = 16)

case object Lab3Key extends Field[Lab3Parameters]

class Lab3BrPredictor(
    fetch_width: Int,
    history_length: Int=9)(implicit p: Parameters)
      extends BrPredictor(fetch_width, history_length)(p)
{
  require (coreInstBytes == 4)
  require (fetch_width == 1)

  // Predictor state: Table of 2-bit counters initialized to 0
  // choice table indexed by global history (9 bits)
  val choice_counters = Reg(
    init = Vec(Seq.fill(512) { UInt("b00", width = 2) }))
  // global counters indexed by global history (9 bits)
  val global_counters = Reg(
    init = Vec(Seq.fill(512) { UInt("b00", width = 2) }))
  // local counters indexed by PC (7 bits)
  val local_counters = Reg(
    init = Vec(Seq.fill(128) { UInt("b00", width = 2) }))

  // pause prediction
  val stall = !io.resp.ready

  // index into the table to get the count
  val s1_pc = io.req_pc
  val s1_ghistory = this.ghistory
  val s1_r_idx = s1_pc >> UInt(log2Ceil(coreInstBytes))
  val s1_choice_count = RegEnable(choice_counters(s1_ghistory), !stall)
  val s1_use_global = s1_choice_count(1)
  val s2_count = Mux( s1_use_global,
    RegEnable(global_counters(s1_ghistory), !stall),
    RegEnable(local_counters(s1_r_idx), !stall))

  // keep sending predictions as long as not disabled
  io.resp.valid := !this.disable_bpd
  // prediction is the upper bit of two-bit counter
  io.resp.bits.takens := s2_count(1)
  // tell the pipeline to save the index for commit
  io.resp.bits.info := RegNext(Cat(s1_r_idx, s1_ghistory))

  // on commit, get relevant info for updates
  val commit_s1_en = this.commit.valid
  val commit_s1_idx = this.commit.bits.info.info >> 9           // leftmost 7 bits are pc index
  val commit_s1_history = this.commit.bits.info.info(8, 0)      // rightmost 9 bits are global history
  val commit_s1_taken = this.commit.bits.ctrl.taken(0)          // taken?
  val commit_s1_mispredict = this.commit.bits.ctrl.mispredicted // mispredicted?

  // index into table to get previous state
  val commit_s2_idx = RegEnable(commit_s1_idx, commit_s1_en)
  val commit_s2_history = RegEnable(commit_s1_history, commit_s1_en)
  val commit_s2_mispredict = commit_s1_mispredict(0)
  val commit_s2_choice_count = RegEnable(choice_counters(commit_s2_history), commit_s1_en)
  val commit_s2_local_count = RegEnable(local_counters(commit_s2_idx), commit_s1_en)
  val commit_s2_global_count = RegEnable(global_counters(commit_s2_history), commit_s1_en)
  val commit_s2_taken = RegEnable(commit_s1_taken, commit_s1_en)
  val commit_s2_en = RegNext(commit_s1_en)
  val commit_use_global = commit_s2_choice_count(1)

  // calculate updated counter value
  val commit_s2_choice_update = Mux(commit_s2_mispredict,
    Mux (commit_use_global,     // if mispredicted: global = -1, local = +1
        Mux(commit_s2_choice_count === "b00".U, commit_s2_choice_count, commit_s2_choice_count - 1.U),
        Mux(commit_s2_choice_count === "b11".U, commit_s2_choice_count, commit_s2_choice_count + 1.U)),
    Mux (commit_use_global,     // if correct prediction: global = +1, local = -1
        Mux(commit_s2_choice_count === "b11".U, commit_s2_choice_count, commit_s2_choice_count + 1.U),
        Mux(commit_s2_choice_count === "b00".U, commit_s2_choice_count, commit_s2_choice_count - 1.U)))
  val commit_s2_local_update = Mux(commit_s2_taken,
    Mux(commit_s2_local_count === "b11".U, commit_s2_local_count, commit_s2_local_count + 1.U),
    Mux(commit_s2_local_count === "b00".U, commit_s2_local_count, commit_s2_local_count - 1.U))
  val commit_s2_global_update = Mux(commit_s2_taken,
    Mux(commit_s2_global_count === "b11".U, commit_s2_global_count, commit_s2_global_count + 1.U),
    Mux(commit_s2_global_count === "b00".U, commit_s2_global_count, commit_s2_global_count - 1.U))

  // write back to table
  when (commit_s2_en) 
  {
    global_counters(commit_s2_history) := commit_s2_global_update
    local_counters(commit_s2_idx) := commit_s2_local_update
    choice_counters(commit_s2_history) := commit_s2_choice_update
  }
}
