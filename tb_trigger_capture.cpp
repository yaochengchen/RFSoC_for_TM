#include <iostream>
#include <hls_stream.h>
#include <ap_int.h>
#include <ap_axi_sdata.h>

typedef ap_axiu<256,0,0,0> axis256_t;

#define CHECK(cond, msg) \
do { \
  if(!(cond)) { \
    std::cout << "TB CHECK FAIL: " << msg << " @line " << __LINE__ << std::endl; \
    return 1; \
  } \
} while(0)

void trigger_capture_hls(
    hls::stream<axis256_t> &s_axis0,
    hls::stream<axis256_t> &s_axis1,
    hls::stream<axis256_t> &s_axis2,
    hls::stream<axis256_t> &s_axis3,

    hls::stream<axis256_t> &m_axis0,
    hls::stream<axis256_t> &m_axis1,
    hls::stream<axis256_t> &m_axis2,
    hls::stream<axis256_t> &m_axis3,

    ap_int<16> thr0, ap_int<16> thr1, ap_int<16> thr2, ap_int<16> thr3,
    ap_uint<8> window_beats,
    ap_uint<32> packet_beats,
    ap_uint<1> clear_trigger,
    ap_uint<1> &trigger_event,
    ap_uint<32> &trigger_beat_index
);

static axis256_t make_beat_one_sample(int16_t v, bool last_in=0){
    axis256_t b;
    b.keep = -1;
    b.last = last_in ? 1 : 0;
    ap_uint<256> d = 0;
    d.range(15,0) = (ap_int<16>)v;   // 只在最低 16bit 填一个 sample，其余为 0
    b.data = d;
    return b;
}

static bool run_one_cycle(
    hls::stream<axis256_t> &in0,
    hls::stream<axis256_t> &in1,
    hls::stream<axis256_t> &in2,
    hls::stream<axis256_t> &in3,
    hls::stream<axis256_t> &out0,
    hls::stream<axis256_t> &out1,
    hls::stream<axis256_t> &out2,
    hls::stream<axis256_t> &out3,
    const axis256_t &b0,
    const axis256_t &b1,
    const axis256_t &b2,
    const axis256_t &b3,
    ap_int<16> thr0, ap_int<16> thr1, ap_int<16> thr2, ap_int<16> thr3,
    ap_uint<8> window_beats,
    ap_uint<32> packet_beats,
    ap_uint<1> clear_trigger,
    ap_uint<1> &trigger_event,
    ap_uint<32> &trigger_beat_index,
    bool expect_last,
    int beat_num_for_log = -1
){
    in0.write(b0);
    in1.write(b1);
    in2.write(b2);
    in3.write(b3);

    trigger_capture_hls(in0,in1,in2,in3, out0,out1,out2,out3,
                        thr0,thr1,thr2,thr3,
                        window_beats, packet_beats, clear_trigger,
                        trigger_event, trigger_beat_index);

    if(out0.empty() || out1.empty() || out2.empty() || out3.empty()){
        std::cout << "TB CHECK FAIL: one of m_axisX is empty at beat=" << beat_num_for_log << std::endl;
        return false;
    }

    axis256_t y0 = out0.read();
    axis256_t y1 = out1.read();
    axis256_t y2 = out2.read();
    axis256_t y3 = out3.read();

    bool ok =
        ((bool)y0.last == expect_last) &&
        ((bool)y1.last == expect_last) &&
        ((bool)y2.last == expect_last) &&
        ((bool)y3.last == expect_last);

    if(!ok){
        std::cout << "TB CHECK FAIL: TLAST mismatch at beat=" << beat_num_for_log
                  << " expect=" << (int)expect_last
                  << " got=[" << (int)y0.last << "," << (int)y1.last << ","
                  << (int)y2.last << "," << (int)y3.last << "]\n";
        return false;
    }
    return true;
}

int main(){
    hls::stream<axis256_t> s0,s1,s2,s3;
    hls::stream<axis256_t> m0,m1,m2,m3;

    ap_int<16> thr = 1000;
    ap_uint<8> window = 8;
    ap_uint<32> pkt_beats = 16;   // 每 16 beat 打一次 TLAST

    ap_uint<1> trig = 0;
    ap_uint<32> trig_idx = 0;

    int global_beat = 0;
    int pkt_phase = 0;

    auto expect_last_now = [&](){
        return (pkt_beats != 0) ? (pkt_phase == (int)pkt_beats - 1) : false;
    };
    auto advance_phase = [&](){
        if (pkt_beats != 0) {
            if (pkt_phase == (int)pkt_beats - 1) pkt_phase = 0;
            else pkt_phase++;
        }
    };

    // ============================================================
    // 0) WARM-UP + clear（第一拍 clear）
    // ============================================================
    for(int b=0;b<16;b++){
        axis256_t z = make_beat_one_sample(0);
        ap_uint<1> clr = (b==0) ? 1 : 0;
        if (clr) pkt_phase = 0; // clear_trigger 会让 DUT pkt_cnt=0

        bool ok = run_one_cycle(s0,s1,s2,s3, m0,m1,m2,m3,
                                z,z,z,z,
                                thr,thr,thr,thr,
                                window, pkt_beats, clr,
                                trig, trig_idx,
                                expect_last_now(),
                                b);
        CHECK(ok, "Warm-up failed");
        advance_phase();
        global_beat++;
    }
    CHECK(trig == 0, "After warm-up, trig should be 0");

    // ============================================================
    // 1) Case1：window 内依次 hit(4路) -> 必须触发
    // ============================================================
    int hit_b0=10, hit_b1=12, hit_b2=15, hit_b3=17;

    trig = 0; trig_idx = 0;
    int observed_trigger_global = -1;

    for(int b=0;b<30;b++){
        axis256_t b0 = make_beat_one_sample((b==hit_b0) ? 2000 : 0);
        axis256_t b1 = make_beat_one_sample((b==hit_b1) ? 2000 : 0);
        axis256_t b2 = make_beat_one_sample((b==hit_b2) ? 2000 : 0);
        axis256_t b3 = make_beat_one_sample((b==hit_b3) ? 2000 : 0);

        bool ok = run_one_cycle(s0,s1,s2,s3, m0,m1,m2,m3,
                                b0,b1,b2,b3,
                                thr,thr,thr,thr,
                                window, pkt_beats, 0,
                                trig, trig_idx,
                                expect_last_now(),
                                b);
        CHECK(ok, "Case1 failed");

        int this_global = global_beat;
        global_beat++;
        advance_phase();

        if(b >= hit_b3 && trig){
            observed_trigger_global = this_global;
            std::cout << "[Case1] Triggered at local b=" << b
                      << " global=" << observed_trigger_global
                      << " reported trig_idx=" << (unsigned)trig_idx << "\n";
            break;
        }
    }

    CHECK(trig == 1, "Case1: trig should be 1 (must trigger)");
    CHECK(observed_trigger_global >= 0, "Case1: did not record observed trigger global beat");

    CHECK(((int)trig_idx >= observed_trigger_global-1) &&
          ((int)trig_idx <= observed_trigger_global+1),
          "Case1: trig_idx not aligned with global beat counter");

    // ============================================================
    // 2) sticky clear 后必须回到 0
    // ============================================================
    {
        axis256_t z = make_beat_one_sample(0);
        pkt_phase = 0;

        bool ok = run_one_cycle(s0,s1,s2,s3, m0,m1,m2,m3,
                                z,z,z,z,
                                thr,thr,thr,thr,
                                window, pkt_beats, 1,
                                trig, trig_idx,
                                expect_last_now(),
                                1000);
        CHECK(ok, "Clear failed");

        advance_phase();
        global_beat++;
        CHECK(trig == 0, "After clear_trigger=1, trig should be 0");
    }

    // ============================================================
    // 3) Case2：超出 window -> 不触发
    // ============================================================
    trig = 0; trig_idx = 0;

    int h0=5, h1=20, h2=21, h3=22;

    for(int b=0;b<40;b++){
        axis256_t b0 = make_beat_one_sample((b==h0) ? 2000 : 0);
        axis256_t b1 = make_beat_one_sample((b==h1) ? 2000 : 0);
        axis256_t b2 = make_beat_one_sample((b==h2) ? 2000 : 0);
        axis256_t b3 = make_beat_one_sample((b==h3) ? 2000 : 0);

        bool ok = run_one_cycle(s0,s1,s2,s3, m0,m1,m2,m3,
                                b0,b1,b2,b3,
                                thr,thr,thr,thr,
                                window, pkt_beats, 0,
                                trig, trig_idx,
                                expect_last_now(),
                                b);
        CHECK(ok, "Case2 failed");

        advance_phase();
        global_beat++;
    }

    std::cout << "[Case2] trig=" << (int)trig << " (expect 0)\n";
    CHECK(trig == 0, "Case2: trig should stay 0 (no trigger expected)");

    std::cout << "ALL TESTS PASSED.\n";
    return 0;
}
