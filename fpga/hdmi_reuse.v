`timescale 1ns/1ps

//复用1路hdmi接口传输两路视频

module hdmi_reuse(
	input                    pclk,
	input					 rst_n,
   input                    vs_in,
   input                    hs_in,
   input                    de_in,
   input             [7:0]  r_in,
   input             [7:0]  g_in,
   input             [7:0]  b_in,

   output reg         		 vs_0,
   output reg        		 hs_0,
   output reg        		 de_0,
   output reg  	  [23:0]  image_0,
   output reg         		 vs_1,
   output reg          		 hs_1,
   output reg         		 de_1,
   output reg   	  [23:0]  image_1
);

reg			   reuse_signal;
reg 		      vs_r;
reg			   hs_r;
reg			   de_r;
reg	 [23:0]	image_r;

always@(posedge pclk or negedge rst_n)begin
	if(!rst_n)
		reuse_signal	<=	1'b0;
	else if((vs_in == 1'b1) && (vs_r == 1'b0))
		reuse_signal	<=	~reuse_signal;
   else
      reuse_signal   <= reuse_signal;
end 

always@(posedge pclk or negedge rst_n)begin
	if(!rst_n)begin
		hs_0	<=	1'b0;
		hs_1	<=	1'b0;
		hs_r	<=	1'b0;
		image_0	<=	24'd0;
		image_1	<=	24'd0;
		image_r	<=	24'd0;
	end else begin
		hs_0	<=	hs_r;
		hs_1	<=	hs_r;
		hs_r	<=	hs_in;
		image_0	<=	image_r;
		image_1	<=	image_r;
		image_r	<=	{r_in, g_in, b_in};
	end
end

always@(posedge pclk or negedge rst_n)begin
   if(!rst_n)begin
      vs_r  <= 1'b0;
      de_r  <= 1'b0;
   end else begin
      vs_r  <= vs_in;
      de_r  <= de_in;
   end
end

always@(posedge pclk or negedge rst_n)begin
	if(!rst_n)begin
		vs_0	 <=   1'b0;
      vs_1	 <=   1'b0;
      de_0   <=	1'b0;
      de_1   <=	1'b0;
   end else if(reuse_signal)begin
      vs_0   <=   1'b0;
      de_0   <=   1'b0;
      vs_1   <=   vs_r;
      de_1   <=   de_r;  
   end else begin
      vs_0   <=   vs_r;
      de_0   <=   de_r; 
      vs_1	 <=   1'b0;
      de_1   <=	1'b0;
   end
end

endmodule