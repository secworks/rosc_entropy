//======================================================================
//
// rosc_entropy_core.v
// -------------------
// Digitial ring oscillator based entropy generation core.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2014, Secworks Sweden AB
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the following
// conditions are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

module rosc_entropy_core(
                         // Clock and reset.
                         input wire           clk,
                         input wire           reset_n,

                         input wire [31 : 0]  opa,
                         input wire [31 : 0]  opb,

                         input wire           update,

                         output wire [31 : 0] rnd,
                         output wire          rnd_valid
                        );


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [31 : 0] shift_reg;
  reg          shift_we;
  reg [31 : 0] rnd_reg;
  reg [4 : 0]  bit_ctr_reg;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg          bit_new;

  wire [7 : 0] dout01;
  wire [7 : 0] dout02;
  wire [7 : 0] dout03;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign rnd = rnd_reg;


  //----------------------------------------------------------------
  // module instantiations.
  //----------------------------------------------------------------
  genvar i;
  generate
    for(i = 0 ; i < 8 ; i = i + 1) begin: oscillators
      bp_osc #(.WIDTH(1)) osc01(.clk(clk),
                                .reset_n(reset_n),
                                .opa(opa[0]),
                                .opb(opb[0]),
                                .dout(dout01[i])
                               );

      bp_osc #(.WIDTH(2)) osc02(.clk(clk),
                                .reset_n(reset_n),
                                .opa(opa[1 : 0]),
                                .opb(opb[1 : 0]),
                                .dout(dout02[i])
                               );

      bp_osc #(.WIDTH(3)) osc03(.clk(clk),
                                .reset_n(reset_n),
                                .opa(opa[2 : 0]),
                                .opb(opb[2 : 0]),
                                .dout(dout03[i])
                               );
    end
  endgenerate


  //----------------------------------------------------------------
  // reg_update
  //
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with synchronous
  // active low reset.
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin
      if (!reset_n)
        begin
          shift_reg   <= 32'h00000000;
          rnd_reg     <= 32'h00000000;
          bit_ctr_reg <= 5'h00;
        end
      else
        begin
          if (update)
            begin
              shift_reg   <= {shift_reg[30 : 0], bit_new};
              bit_ctr_reg <= bit_ctr_reg + 1'b1;
            end

          if (bit_ctr_reg == 5'h1f)
            begin
              rnd_reg <= shift_reg;
            end
        end
    end // reg_update


  //----------------------------------------------------------------
  // rnd_gen
  //
  // Logic that implements the actual random bit value generator
  // by mixing the oscillator outputs.
  //----------------------------------------------------------------
  always @*
    begin : rnd_gen
      reg osc1_mix;
      reg osc2_mix;
      reg osc3_mix;

      osc1_mix = ^dout01;
      osc2_mix = ^dout02;
      osc3_mix = ^dout03;

      bit_new = osc1_mix ^ osc2_mix ^ osc3_mix;
    end
endmodule // rosc_entropy_core

//======================================================================
// EOF rosc_entropy_core.v
//======================================================================
