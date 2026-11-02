//CLAHE算法顶层模块（1280x720@30fps，8x8=64 个 tile）

/////////////////////////////////////////////////////////
//每个 tile 尺寸 160*90 = 14400 pixels，截断单个 tile 内 //
//数量大于 14400*cliplimit/256 的灰度值，默认           //
//cliplimit=2.0，可通过上位机接收 cliplimit 值          //
//                                                      //
//直方图存储：8 块 2048x16bit RAM（每列一块）           //
//映射表：64 块 256x8bit 分布式 RAM                     //
/////////////////////////////////////////////////////////

module clahe_top(
	input					pclk,
	input					rst_n,
	input			[4:0]	cliplimitx8,//>>3得到cliplimit，顶层默认cliplimit=2.0，
	input					interp_mode,//0=整幅 8x8 双线性(默认)；1=4 区(640x360)各 4x4
	input			[23:0]	i_image,	//YUV原始图像
	input					i_vs,
	input					i_hs,
	input					i_de,
	input			[10:0]	i_hc,
	input			[9:0]	i_vc,
	output	wire	[23:0]	o_image,	//RGB
	output	wire			o_vs,
	output	wire			o_hs,
    output	wire	[10:0]	o_hc,
	output	wire	[9:0]	o_vc,
	output	wire			o_de
   );

wire	[10:0]	x_pos;          
wire	[9:0]	y_pos;          
wire	[7:0]	tile_x_pos;		
wire	[6:0]	tile_y_pos;		
wire	[2:0]	tile_x_index;	
wire	[2:0]	tile_y_index;	
wire	[5:0]	tile_index;		
wire			vs_r;
wire            hs_r;
wire			de_r;
wire    [10:0]  hc_r;
wire    [9:0]   vc_r;
wire	[23:0]	yuv_r;

position	position_inst(
	.pclk				(pclk),
	.rst_n				(rst_n),
	.i_vs				(i_vs),
	.i_de				(i_de),
	.x_pos      		(x_pos),
    .y_pos              (y_pos),
    .tile_x_pos	        (tile_x_pos),
    .tile_y_pos	        (tile_y_pos),
    .tile_x_index       (tile_x_index),
    .tile_y_index       (tile_y_index),
    .tile_index	        (tile_index)
);

wire	            redistri_done   ;
wire                cdf_done        ;
wire    [5:0]       rd_tile_index_1 ;
wire    [7:0]       rd_addr_1       ;
wire    [15:0]      rd_data_1       ;
wire    [5:0]       rd_tile_index_2 ;
wire    [7:0]       rd_addr_2       ;
wire    [15:0]      rd_data_2       ;
wire    [5:0]       rd_tile_index_3 ;
wire    [7:0]       rd_addr_3       ;
wire    [15:0]      rd_data_3       ;
wire    [5:0]       rd_tile_index_4 ;
wire    [7:0]       rd_addr_4       ;
wire    [15:0]      rd_data_4       ;
wire    [5:0]       rd_tile_index_5 ;
wire    [7:0]       rd_addr_5       ;
wire    [15:0]      rd_data_5       ;
wire    [5:0]       rd_tile_index_6 ;
wire    [7:0]       rd_addr_6       ;
wire    [15:0]      rd_data_6       ;
wire    [5:0]       rd_tile_index_7 ;
wire    [7:0]       rd_addr_7       ;
wire    [15:0]      rd_data_7       ;
wire    [5:0]       rd_tile_index_8 ;
wire    [7:0]       rd_addr_8       ;
wire    [15:0]      rd_data_8       ;

hist_ram_ctrl hist_ram_ctrl_inst (
    .pclk             (pclk),
    .rst_n            (rst_n),
    .cliplimitx8      (cliplimitx8),
    .i_y              (i_image[23:16]),
    .i_vs             (i_vs),
    .i_de             (i_de),
    .i_tile_index     (tile_index),
    .i_x_pos          (x_pos),
    .i_y_pos          (y_pos),
    .redistri_done    (redistri_done),
    .cdf_done         (cdf_done),
    .rd_tile_index_1  (rd_tile_index_1  ),
    .rd_addr_1        (rd_addr_1        ),
    .rd_data_1        (rd_data_1        ),
    .rd_tile_index_2  (rd_tile_index_2  ),
    .rd_addr_2        (rd_addr_2        ),
    .rd_data_2        (rd_data_2        ),
    .rd_tile_index_3  (rd_tile_index_3  ),
    .rd_addr_3        (rd_addr_3        ),
    .rd_data_3        (rd_data_3        ),
    .rd_tile_index_4  (rd_tile_index_4  ),
    .rd_addr_4        (rd_addr_4        ),
    .rd_data_4        (rd_data_4        ),
    .rd_tile_index_5  (rd_tile_index_5  ),
    .rd_addr_5        (rd_addr_5        ),
    .rd_data_5        (rd_data_5        ),
    .rd_tile_index_6  (rd_tile_index_6  ),
    .rd_addr_6        (rd_addr_6        ),
    .rd_data_6        (rd_data_6        ),
    .rd_tile_index_7  (rd_tile_index_7  ),
    .rd_addr_7        (rd_addr_7        ),
    .rd_data_7        (rd_data_7        ),
    .rd_tile_index_8  (rd_tile_index_8  ),
    .rd_addr_8        (rd_addr_8        ),
    .rd_data_8        (rd_data_8        )
);

wire    [5:0]       lut_tile_index_1 ;
wire    [7:0]       lut_addr_1       ;
wire                lut_index_val_1  ;
wire    [7:0]       lut_data_1       ;
wire    [5:0]       lut_tile_index_2 ;
wire    [7:0]       lut_addr_2       ;
wire                lut_index_val_2  ;
wire    [7:0]       lut_data_2       ;
wire    [5:0]       lut_tile_index_3 ; 
wire    [7:0]       lut_addr_3       ;
wire                lut_index_val_3  ;
wire    [7:0]       lut_data_3       ;
wire    [5:0]       lut_tile_index_4 ;
wire    [7:0]       lut_addr_4       ;
wire                lut_index_val_4  ;
wire    [7:0]       lut_data_4       ; 

cdf_lut u_cdf_lut(
    .pclk             (pclk             ),
    .rst_n            (rst_n            ),
    .redistri_done    (redistri_done    ),
    .cdf_done         (cdf_done         ),
    .rd_tile_index_1  (rd_tile_index_1  ),
    .rd_addr_1        (rd_addr_1        ),
    .rd_data_1        (rd_data_1        ),
    .rd_tile_index_2  (rd_tile_index_2  ),
    .rd_addr_2        (rd_addr_2        ),
    .rd_data_2        (rd_data_2        ),
    .rd_tile_index_3  (rd_tile_index_3  ),
    .rd_addr_3        (rd_addr_3        ),
    .rd_data_3        (rd_data_3        ),
    .rd_tile_index_4  (rd_tile_index_4  ),
    .rd_addr_4        (rd_addr_4        ),
    .rd_data_4        (rd_data_4        ),
    .rd_tile_index_5  (rd_tile_index_5  ),
    .rd_addr_5        (rd_addr_5        ),
    .rd_data_5        (rd_data_5        ),
    .rd_tile_index_6  (rd_tile_index_6  ),
    .rd_addr_6        (rd_addr_6        ),
    .rd_data_6        (rd_data_6        ),
    .rd_tile_index_7  (rd_tile_index_7  ),
    .rd_addr_7        (rd_addr_7        ),
    .rd_data_7        (rd_data_7        ),
    .rd_tile_index_8  (rd_tile_index_8  ),
    .rd_addr_8        (rd_addr_8        ),
    .rd_data_8        (rd_data_8        ),
    .lut_tile_index_1 (lut_tile_index_1 ),
    .lut_addr_1       (lut_addr_1       ),
    .lut_index_val_1  (lut_index_val_1  ),
    .lut_data_1       (lut_data_1       ),
    .lut_tile_index_2 (lut_tile_index_2 ),
    .lut_addr_2       (lut_addr_2       ),
    .lut_index_val_2  (lut_index_val_2  ),
    .lut_data_2       (lut_data_2       ),
    .lut_tile_index_3 (lut_tile_index_3 ),
    .lut_addr_3       (lut_addr_3       ),
    .lut_index_val_3  (lut_index_val_3  ),
    .lut_data_3       (lut_data_3       ),
    .lut_tile_index_4 (lut_tile_index_4 ),
    .lut_addr_4       (lut_addr_4       ),
    .lut_index_val_4  (lut_index_val_4  ),
    .lut_data_4       (lut_data_4       )
);

bil_inter u_bil_inter(
    .pclk             (pclk             ),
    .rst_n            (rst_n            ),
    .i_vs             (i_vs             ),
    .i_hs             (i_hs             ),
    .i_hc             (i_hc             ),
    .i_vc             (i_vc             ),
    .i_de             (i_de             ),
    .i_image          (i_image          ),
    .x_pos            (x_pos            ),
    .y_pos            (y_pos            ),
    .tile_x_index     (tile_x_index     ),
    .tile_y_index     (tile_y_index     ),
    .tile_x_pos       (tile_x_pos       ),
    .tile_y_pos       (tile_y_pos       ),
    .tile_index       (tile_index       ),
    .interp_mode      (interp_mode      ),
    .lut_tile_index_1 (lut_tile_index_1 ),
    .lut_addr_1       (lut_addr_1       ),
    .lut_index_val_1  (lut_index_val_1  ),
    .lut_data_1       (lut_data_1       ),
    .lut_tile_index_2 (lut_tile_index_2 ),
    .lut_addr_2       (lut_addr_2       ),
    .lut_index_val_2  (lut_index_val_2  ),
    .lut_data_2       (lut_data_2       ),
    .lut_tile_index_3 (lut_tile_index_3 ),
    .lut_addr_3       (lut_addr_3       ),
    .lut_index_val_3  (lut_index_val_3  ),
    .lut_data_3       (lut_data_3       ),
    .lut_tile_index_4 (lut_tile_index_4 ),
    .lut_addr_4       (lut_addr_4       ),
    .lut_index_val_4  (lut_index_val_4  ),
    .lut_data_4       (lut_data_4       ),
    .o_image          (yuv_r            ),
    .o_vs             (vs_r             ),
    .o_hs             (hs_r             ),
    .o_de             (de_r             ),
    .o_hc             (hc_r             ),
    .o_vc             (vc_r             )
);

yuv2rgb u_yuv2rgb(
    .pclk     (pclk     ),
    .rst_n    (rst_n    ),
    .i_vs     (vs_r     ),
    .i_hs     (hs_r     ),
    .i_de     (de_r     ),
    .i_hc     (hc_r     ),
    .i_vc     (vc_r     ), 
    .i_yuv444 (yuv_r    ),
    .o_vs     (o_vs     ),
    .o_hs     (o_hs     ),
    .o_de     (o_de     ),
    .o_hc     (o_hc     ),
    .o_vc     (o_vc     ),
    .o_rgb565 (o_image  )
);

endmodule
