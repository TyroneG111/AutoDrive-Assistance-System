module mode_ctrl
(
    input               clk,          // 系统时钟
    input               rst_n,        // 低电平复位
    input               key_pulse,    // 按键触发脉冲（单周期有效）
    output reg [4:0]    onehot_out    // 5位独热码输出，默认00001
);

//==================== 独热码循环切换逻辑 ====================
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        // 复位默认：最低位为1，其余为0 → 5'b00001
        onehot_out <= 5'b00001;
    end
    else begin
        if(key_pulse) begin
            // 循环左移一位：最高位移到最低位，其余左移
            // 切换顺序：00001 → 00010 → 00100 → 01000 → 10000 → 00001
            onehot_out <= {onehot_out[3:0], onehot_out[4]};
        end
        else begin
            onehot_out <= onehot_out;
        end
    end
end

endmodule
