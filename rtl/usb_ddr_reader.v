// Fetches the USB capture stream from a ring buffer in DDR3 that the ARM side
// fills, so audio no longer depends on the Linux main loop's SPI polling.
//
// DDR3 layout, starting at BASE (64 bit words):
//   word 0x000          header: [31:0] frames written by the ARM (wraps at 2^32),
//                       [63:32] magic 0x55534244 while the ARM feeds the ring
//   word 0x200 ...      ring of 32768 words, two frames each (low half first),
//                       frame = {left[15:0], right[15:0]}
//
// The module runs on the 114 MHz DDR3 clock and hands frames to usb_sampler on
// clk_sys through a toggle handshake.

module usb_ddr_reader #(parameter [28:0] BASE = 29'h03FE8000)
(
	input             clk,         // DDR3 clock
	input             rst,

	output reg [28:0] ddr_addr,
	output reg        ddr_rd,
	input             ddr_busy,
	input      [63:0] ddr_dout,
	input             ddr_dout_ready,

	input             fifo_req,    // usb_sampler wants data (clk_sys domain)
	input             deep,        // Toccata recording (clk_sys domain)

	input             clk_sys,
	output reg        fw,
	output reg [31:0] fdata
);

localparam [31:0] MAGIC        = 32'h55534244;  // "USBD" in header[63:32]
localparam [31:0] KEEP         = 32'd1024;
localparam [31:0] MAXLAG_NORM  = 32'd3072;
localparam [31:0] MAXLAG_DEEP  = 32'd61440;

reg       ack = 0;
reg [1:0] req_s = 0, deep_s = 0, ack_s = 0;
always @(posedge clk) begin
	req_s  <= {req_s[0],  fifo_req};
	deep_s <= {deep_s[0], deep};
	ack_s  <= {ack_s[0],  ack};
end

reg        init;
reg [31:0] rptr, wptr;
reg        wvalid;
reg [12:0] poll;
reg [63:0] word;
reg        tgl;
reg [19:0] tmo;
reg [2:0]  st;

localparam S_IDLE = 0, S_HDR_RD = 1, S_HDR_WAIT = 2, S_DAT_RD = 3, S_DAT_WAIT = 4;

wire [31:0] lag      = wptr - rptr;
wire        lag_neg  = lag[31];
wire [31:0] maxlag   = deep_s[1] ? MAXLAG_DEEP : MAXLAG_NORM;
wire        data_rdy = wvalid && !lag_neg && lag >= 32'd2 && req_s[1] && (ack_s[1] == tgl);
wire        need_hdr = (!wvalid || lag_neg || lag < 32'd2) ? poll[8] : poll[12];

wire [31:0] new_w   = ddr_dout[31:0];
wire [31:0] new_lag = new_w - rptr;

always @(posedge clk or posedge rst) begin
	if (rst) begin
		st     <= S_IDLE;
		ddr_rd <= 0;
		init   <= 0;
		wvalid <= 0;
		poll   <= 0;
		tgl    <= 0;
		rptr   <= 0;
		wptr   <= 0;
		tmo    <= 0;
	end
	else begin
		if (poll != 13'h1FFF) poll <= poll + 1'd1;
		if (ddr_rd & ~ddr_busy) ddr_rd <= 0;

		case (st)
		S_IDLE: begin
			tmo <= 0;
			if (data_rdy) begin
				ddr_addr <= BASE + 29'h200 + {14'd0, rptr[15:1]};
				ddr_rd   <= 1;
				st       <= S_DAT_RD;
			end
			else if (need_hdr) begin
				ddr_addr <= BASE;
				ddr_rd   <= 1;
				poll     <= 0;
				st       <= S_HDR_RD;
			end
		end

		S_HDR_RD: if (ddr_rd & ~ddr_busy) st <= S_HDR_WAIT;
		S_DAT_RD: if (ddr_rd & ~ddr_busy) st <= S_DAT_WAIT;

		S_HDR_WAIT: begin
			tmo <= tmo + 1'd1;
			if (ddr_dout_ready) begin
				// Without the magic value the ARM side is not feeding the ring (older Main, device gone).
				if (ddr_dout[63:32] != MAGIC) begin
					wvalid <= 0;
					init   <= 0;
				end
				else begin
					wptr   <= new_w;
					wvalid <= 1;
					// The ring is only valid within one ring length behind the write position.
					if (!init) begin
						rptr <= (new_w - KEEP) & ~32'd1;
						init <= 1;
					end
					else if (new_lag[31] || new_lag > maxlag)
						rptr <= (new_w - KEEP) & ~32'd1;
				end
				st <= S_IDLE;
			end
			else if (tmo[19]) st <= S_IDLE;
		end

		S_DAT_WAIT: begin
			tmo <= tmo + 1'd1;
			if (ddr_dout_ready) begin
				word <= ddr_dout;
				tgl  <= ~tgl;
				rptr <= rptr + 32'd2;
				st   <= S_IDLE;
			end
			else if (tmo[19]) st <= S_IDLE;
		end

		default: st <= S_IDLE;
		endcase
	end
end

// clk_sys side: write both frames of a fetched word, then acknowledge.
reg [2:0]  tgl_s = 0;
reg [31:0] hi;
reg [1:0]  ph = 0;
always @(posedge clk_sys) begin
	tgl_s <= {tgl_s[1:0], tgl};
	fw    <= 0;

	case (ph)
	0: if (tgl_s[2] ^ tgl_s[1]) begin
		fdata <= word[31:0];
		hi    <= word[63:32];
		fw    <= 1;
		ph    <= 1;
	end
	1: begin
		fdata <= hi;
		fw    <= 1;
		ph    <= 2;
	end
	2: begin
		ack <= tgl_s[1];
		ph  <= 0;
	end
	default: ph <= 0;
	endcase
end

endmodule

// Shares one DDR3 read port between two single-beat read masters. Each master
// holds its read request until it sees busy low; the grant is held until the
// data has returned.
module ddr_rd_mux2
(
	input             clk,
	input             rst,

	input      [28:0] a_addr,
	input             a_rd,
	output            a_busy,
	output            a_ready,

	input      [28:0] b_addr,
	input             b_rd,
	output            b_busy,
	output            b_ready,

	output     [28:0] m_addr,
	output            m_rd,
	input             m_busy,
	input             m_ready
);

reg granted, who, pend;

always @(posedge clk or posedge rst) begin
	if (rst) begin
		granted <= 0;
		who     <= 0;
		pend    <= 0;
	end
	else if (!granted) begin
		if (a_rd) begin granted <= 1; who <= 0; end
		else if (b_rd) begin granted <= 1; who <= 1; end
	end
	else if (!pend) begin
		if (m_rd & ~m_busy) pend <= 1;
	end
	else if (m_ready) begin
		granted <= 0;
		pend    <= 0;
	end
end

wire live = granted & ~pend;
assign m_addr  = who ? b_addr : a_addr;
assign m_rd    = live & (who ? b_rd : a_rd);
assign a_busy  = ~(live & ~who) | m_busy;
assign b_busy  = ~(live &  who) | m_busy;
assign a_ready = m_ready & pend & ~who;
assign b_ready = m_ready & pend &  who;

endmodule
