//======================================================================
//  position.v  —— 像素坐标 / tile 解码
//----------------------------------------------------------------------
//  图像 1280x720，划分为 8x8 = 64 个 tile，每个 tile 160x90 = 14400 像素。
//  tile_index = tile_y_index*8 + tile_x_index  （0 ~ 63）
//
//  * 坐标推进直接以 i_de 为像素节拍（每像素有效一次），同时适配两种行时序：
//      - 2 pclk/像素：i_de 交替，每 2 拍出现一次高电平（原 OV5640 8bit 拼接时序）
//      - 1 pclk/像素：i_de 在有效区内恒高，每拍一个像素
//    i_de 高电平那一拍的 x_pos / y_pos 取值与原 phase 方案完全一致。
//  * tile 索引为纯组合译码，与 x_pos / y_pos 同拍。
//  * tile 宽 160 / 高 90 都不是 2 的幂，因此索引改用比较链译码；
//    tile 内坐标 = 坐标 - 索引*tile 尺寸，常数乘用移位加法实现
//    （160 = 128+32，90 = 64+16+8+2），不出现乘法器。
//  =>  i_de & (x_pos==1279) & (y_pos==719) 每帧恰好有效 1 拍。
//======================================================================

module position(
	input					pclk,
	input					rst_n,
	input					i_vs,
	input					i_de,
	output	reg		[10:0]	x_pos,			//整幅图横坐标 0~1279
	output	reg		[9:0]	y_pos,			//整幅图纵坐标 0~719
	output	wire	[7:0]	tile_x_pos,		//tile内横坐标 0~159
	output	wire	[6:0]	tile_y_pos,		//tile内纵坐标 0~89
	output	wire	[2:0]	tile_x_index,	//横向tile序号 0~7
	output	wire	[2:0]	tile_y_index,	//纵向tile序号 0~7
	output	wire	[5:0]	tile_index		//tile序号 0~63
   );

localparam	[10:0]	X_LAST	=	11'd1279;
localparam	[9:0]	Y_LAST	=	10'd719;

//------------------------------------------------------------------
// 像素推进使能 adv_pix
//   i_de 在两种行时序下都恰好“每像素有效一次”，直接用它推进坐标：
//     * 2 pclk/像素：i_de 交替，每 2 拍出现一次高电平；
//     * 1 pclk/像素：i_de 恒高，每拍一个像素。
//   对 2 pclk/像素时序，i_de 高电平那一拍的 x_pos/y_pos 与原 phase 方案一致，
//   行为不变；对 1 pclk/像素时序，每行恰好推进 1280 次，行末正确回绕。
//------------------------------------------------------------------
wire	adv_pix	=	i_de;

//------------------------------------------------------------------
// x_pos：每像素 +1，行末(1279)在像素结束时回绕
//------------------------------------------------------------------
always@(posedge pclk or negedge rst_n)begin
	if(!rst_n)
		x_pos	<=	11'd0;
	else if(i_vs)
		x_pos	<=	11'd0;
	else if(adv_pix)begin
		if(x_pos == X_LAST)
			x_pos	<=	11'd0;
		else
			x_pos	<=	x_pos + 11'd1;
	end
end

//------------------------------------------------------------------
// y_pos：行末(且像素结束时) +1，场末(719)回绕为 0
//------------------------------------------------------------------
always@(posedge pclk or negedge rst_n)begin
	if(!rst_n)
		y_pos	<=	10'd0;
	else if(i_vs)
		y_pos	<=	10'd0;
	else if(adv_pix && (x_pos == X_LAST))begin
		if(y_pos == Y_LAST)
			y_pos	<=	10'd0;
		else
			y_pos	<=	y_pos + 10'd1;
	end
end

//------------------------------------------------------------------
// tile 解码：纯组合，与 x_pos / y_pos 同拍
//------------------------------------------------------------------
//横向：每 160 列一个 tile，共 8 个（比较链，避免除法）
assign	tile_x_index	=	(x_pos < 11'd160 ) ? 3'd0 :
						(x_pos < 11'd320 ) ? 3'd1 :
						(x_pos < 11'd480 ) ? 3'd2 :
						(x_pos < 11'd640 ) ? 3'd3 :
						(x_pos < 11'd800 ) ? 3'd4 :
						(x_pos < 11'd960 ) ? 3'd5 :
						(x_pos < 11'd1120) ? 3'd6 : 3'd7;

//纵向：每 90 行一个 tile，共 8 个（比较链，避免除法）
assign	tile_y_index	=	(y_pos < 10'd90 ) ? 3'd0 :
						(y_pos < 10'd180) ? 3'd1 :
						(y_pos < 10'd270) ? 3'd2 :
						(y_pos < 10'd360) ? 3'd3 :
						(y_pos < 10'd450) ? 3'd4 :
						(y_pos < 10'd540) ? 3'd5 :
						(y_pos < 10'd630) ? 3'd6 : 3'd7;

//tile 内横坐标 = x_pos - tile_x_index*160 （160 = 128 + 32，移位加法）
wire	[10:0]	x_base	=	({8'd0,tile_x_index} << 7) + ({8'd0,tile_x_index} << 5);
assign	tile_x_pos	=	x_pos - x_base;							//0~159

//tile 内纵坐标 = y_pos - tile_y_index*90 （90 = 64 + 16 + 8 + 2，移位加法）
wire	[9:0]	y_base	=	({7'd0,tile_y_index} << 6) + ({7'd0,tile_y_index} << 4)
					  + ({7'd0,tile_y_index} << 3) + ({7'd0,tile_y_index} << 1);
assign	tile_y_pos	=	y_pos - y_base;							//0~89

//tile 序号 = tile_y_index*8 + tile_x_index （*8 即移位）
assign	tile_index	=	{tile_y_index,3'd0} + {3'd0,tile_x_index};

endmodule
