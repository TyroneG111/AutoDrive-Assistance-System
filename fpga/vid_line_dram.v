`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: vid_line_dram
// Description: 视频行缓存 真双口 RAM 封装
//   对外提供简单接口, 内部例化 Pango "DRM Based Dual Port RAM" IP。
//   IP 生成参数(请在 PDS IPC 中按下述配置生成, 实例/模块名: vid_line_dram_ip):
//     - 类型: Module/Memory/DRM -> DRM Based Dual Port RAM
//     - 数据位宽: 24        (真双口, 独立时钟)
//     - 地址位宽: 13  (深度 8192)
//     - A 端口: 写 (时钟 = vid_pclk)   B 端口: 读 (时钟 = rgmii_clk)
//     - A 端口写模式: Normal_Write;  读输出寄存(rd_oce) 关闭 -> 读延迟 1 拍
//     - 不要勾选 "Enable Byte Write" (本设计每次写/读都是整 24bit 字, 无需字节掩码;
//       且 Byte Write 仅对 8bit 位宽有意义)
//     - 请保持端口名与本文件内部例化一致(a_/b_ 前缀)。若生成名称不同,
//       仅需修改本文件内部的例化名/端口名。
//   备注: 若 IPC 生成的模块名不是 vid_line_dram_ip, 请在此处改名即可。
//////////////////////////////////////////////////////////////////////////////////

module vid_line_dram #(
    parameter AW = 13,
    parameter DW = 24
)(
    input               wr_clk,
    input               wr_en,
    input  [AW-1:0]     wr_addr,
    input  [DW-1:0]     wr_data,

    input               rd_clk,
    input  [AW-1:0]     rd_addr,
    output [DW-1:0]     rd_data
);

    vid_line_dram_ip u_vid_line_dram_ip (
        .a_clk         (wr_clk),
        .a_clk_en      (1'b1),
        .a_rst         (1'b0),
        .a_wr_en       (wr_en),
        .a_addr        (wr_addr),
        .a_wr_data     (wr_data),
        .a_rd_data     (),

        .b_clk         (rd_clk),
        .b_clk_en      (1'b1),
        .b_rst         (1'b0),
        .b_wr_en       (1'b0),
        .b_addr        (rd_addr),
        .b_wr_data     ({DW{1'b0}}),
        .b_rd_data     (rd_data)
    );

endmodule
