module dri_staus(
	input				clk,
	input				rst_n,
	input				key_1,
	input				key_2,
	input				key_3,
	input				key_4,
	input				key_5,
	output		[4:0]	staus
   );

wire key1_stable;
wire key2_stable;
wire key3_stable;
wire key4_stable;
wire key5_stable;

key_debounce #(
	.CLK_FREQ	(125_000_000),
	.DEBOUNCE_MS(5)
) u1_key_debounc (
    .clk        (clk        ),
    .rst_n      (rst_n      ),
    .key_in     (key_1      ),
    .key_stable (key1_stable),
    .key_press  (           )
);

key_debounce #(
	.CLK_FREQ	(125_000_000),
	.DEBOUNCE_MS(5)
) u2_key_debounc (
    .clk        (clk        ),
    .rst_n      (rst_n      ),
    .key_in     (key_2      ),
    .key_stable (key2_stable),
    .key_press  (           )
);

key_debounce #(
	.CLK_FREQ	(125_000_000),
	.DEBOUNCE_MS(5)
) u3_key_debounc (
    .clk        (clk        ),
    .rst_n      (rst_n      ),
    .key_in     (key_3      ),
    .key_stable (key3_stable),
    .key_press  (           )
);

key_debounce #(
	.CLK_FREQ	(125_000_000),
	.DEBOUNCE_MS(5)
) u4_key_debounc (
    .clk        (clk        ),
    .rst_n      (rst_n      ),
    .key_in     (key_4      ),
    .key_stable (key4_stable),
    .key_press  (           )
);

key_debounce #(
	.CLK_FREQ	(125_000_000),
	.DEBOUNCE_MS(5)
) u5_key_debounc (
    .clk        (clk        ),
    .rst_n      (rst_n      ),
    .key_in     (key_5      ),
    .key_stable (key5_stable),
    .key_press  (           )
);

assign	staus = {~key1_stable, ~key2_stable, ~key3_stable, ~key4_stable, ~key5_stable};

endmodule