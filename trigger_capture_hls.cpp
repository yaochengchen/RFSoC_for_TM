#include <ap_int.h>
#include <ap_axi_sdata.h>
#include <hls_stream.h>

typedef ap_axiu<256,0,0,0> axis256_t;

static inline ap_int<16> abs16(ap_int<16> v){
#pragma HLS INLINE
    return (v < 0) ? (ap_int<16>)(-v) : v;
}

static inline ap_uint<1> any_hit_256(const ap_uint<256>& d, ap_int<16> thr){
#pragma HLS INLINE
    ap_uint<1> hit = 0;
    for(int i=0;i<16;i++){
    #pragma HLS UNROLL
        ap_int<16> s = d.range(i*16+15, i*16);
        hit |= (abs16(s) > thr);
    }
    return hit;
}

// 4x AXIS in + 4x AXIS out (single clock domain on the HLS side)
void trigger_capture_hls(
    hls::stream<axis256_t> &s_axis0,
    hls::stream<axis256_t> &s_axis1,
    hls::stream<axis256_t> &s_axis2,
    hls::stream<axis256_t> &s_axis3,

    hls::stream<axis256_t> &m_axis0,
    hls::stream<axis256_t> &m_axis1,
    hls::stream<axis256_t> &m_axis2,
    hls::stream<axis256_t> &m_axis3,

    ap_int<16> thr0,
    ap_int<16> thr1,
    ap_int<16> thr2,
    ap_int<16> thr3,

    ap_uint<8>  window_beats,         // coincidence window
    ap_uint<32> packet_beats,         // 每 packet_beats 个 beat 打一次 TLAST（0=透传原last）

    ap_uint<1>  clear_trigger,        // PS 写 1 清除 sticky（这里也会顺便清 pkt_cnt）

    ap_uint<1>  &trigger_event,       // sticky
    ap_uint<32> &trigger_beat_index   // 触发发生的 beat 计数
)
{
#pragma HLS INTERFACE axis port=s_axis0
#pragma HLS INTERFACE axis port=s_axis1
#pragma HLS INTERFACE axis port=s_axis2
#pragma HLS INTERFACE axis port=s_axis3

#pragma HLS INTERFACE axis port=m_axis0
#pragma HLS INTERFACE axis port=m_axis1
#pragma HLS INTERFACE axis port=m_axis2
#pragma HLS INTERFACE axis port=m_axis3

#pragma HLS INTERFACE s_axilite port=thr0 bundle=CTRL
#pragma HLS INTERFACE s_axilite port=thr1 bundle=CTRL
#pragma HLS INTERFACE s_axilite port=thr2 bundle=CTRL
#pragma HLS INTERFACE s_axilite port=thr3 bundle=CTRL
#pragma HLS INTERFACE s_axilite port=window_beats bundle=CTRL
#pragma HLS INTERFACE s_axilite port=packet_beats bundle=CTRL
#pragma HLS INTERFACE s_axilite port=clear_trigger bundle=CTRL
#pragma HLS INTERFACE s_axilite port=trigger_event bundle=CTRL
#pragma HLS INTERFACE s_axilite port=trigger_beat_index bundle=CTRL
#pragma HLS INTERFACE ap_ctrl_hs port=return
#pragma HLS INTERFACE s_axilite port=return bundle=CTRL

#pragma HLS PIPELINE II=1

    static ap_uint<8>  c0=0,c1=0,c2=0,c3=0;
    static ap_uint<1>  trig_sticky = 0;
    static ap_uint<32> beat_counter = 0;
    static ap_uint<32> trig_index_latched = 0;
    // TLAST 计数器（全局共用，四路刚性同步）
    static ap_uint<32> pkt_cnt = 0;
if (clear_trigger) {
        trig_sticky = 0;
        pkt_cnt = 0;
    }

    // 读取 4 路输入
    axis256_t in0 = s_axis0.read();
    axis256_t in1 = s_axis1.read();
    axis256_t in2 = s_axis2.read();
    axis256_t in3 = s_axis3.read();

    // ===== TLAST 生成（按每路独立 pkt_cnt 规律）=====
    axis256_t out0 = in0, out1 = in1, out2 = in2, out3 = in3;

    if (packet_beats != 0) {
        ap_uint<1> last_now = (pkt_cnt == (packet_beats - 1));
        out0.last = last_now;
        out1.last = last_now;
        out2.last = last_now;
        out3.last = last_now;

        pkt_cnt = last_now ? (ap_uint<32>)0 : (ap_uint<32>)(pkt_cnt + 1);
    }
    // packet_beats=0：保持原始 last（out 已经等于 in，不用额外处理）

    // 写 4 路输出
    m_axis0.write(out0);
    m_axis1.write(out1);
    m_axis2.write(out2);
    m_axis3.write(out3);

    // ===== Trigger coincidence =====
    ap_uint<1> hit0 = any_hit_256(in0.data, thr0);
    ap_uint<1> hit1 = any_hit_256(in1.data, thr1);
    ap_uint<1> hit2 = any_hit_256(in2.data, thr2);
    ap_uint<1> hit3 = any_hit_256(in3.data, thr3);

    if(hit0) c0 = window_beats; else if(c0>0) c0--;
    if(hit1) c1 = window_beats; else if(c1>0) c1--;
    if(hit2) c2 = window_beats; else if(c2>0) c2--;
    if(hit3) c3 = window_beats; else if(c3>0) c3--;

    if(!trig_sticky && (c0>0 && c1>0 && c2>0 && c3>0)){
        trig_sticky = 1;
        trig_index_latched = beat_counter;
    }

    trigger_event = trig_sticky;
    trigger_beat_index = trig_index_latched;

    beat_counter++;
}
