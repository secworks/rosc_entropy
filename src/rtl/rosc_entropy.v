//======================================================================
//
// rosc_entropy.v
// --------------
// Top level wrapper for the ring oscillator entropy core.
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

module rosc_entropy(
                    input wire           clk,
                    input wire           reset_n,

                    output wire [7 : 0]  debug,
                    input wire           debug_update,

                    input wire           cs,
                    input wire           we,
                    input wire  [7 : 0]  address,
                    input wire  [31 : 0] write_data,
                    output wire [31 : 0] read_data,
                    output wire          error
                   );


  //----------------------------------------------------------------
  // Parameters.
  //----------------------------------------------------------------
  parameter ADDR_CTRL                = 8'h00;
  parameter CTRL_ENABLE_BIT          = 0;

  parameter ADDR_STATUS              = 8'h01;
  parameter STATUS_ENTROPY_VALID_BIT = 0;

  parameter ADDR_OP                  = 8'h08;

  parameter ADDR_ENTROPY             = 8'h10;
  parameter ADDR_RAW                 = 8'h20;
  parameter ADDR_ROSC_OUTPUTS        = 8'h21;

  parameter DEFAULT_OP               = 8'haaaaaaaa;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg          en_reg;
  reg          en_new;
  reg          en_we;

  reg [31 : 0] op_a_reg;
  reg [31 : 0] op_a_new;
  reg          op_a_we;

  reg [31 : 0] op_b_reg;
  reg [31 : 0] op_b_new;
  reg          op_b_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  wire [31 : 0] entropy;

  wire [31 : 0] rnd_data;
  wire          rnd_valid;
  reg           rnd_ack;

  reg [31 : 0]  tmp_read_data;
  reg           tmp_error;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign read_data = tmp_read_data;
  assign error     = tmp_error;


  //----------------------------------------------------------------
  // module instantiations.
  //----------------------------------------------------------------
  rosc_entropy_core core(
                         .clk(clk),
                         .reset_n(reset_n),

                         .enable(en_reg),

                         .opa(op_a_reg),
                         .opb(op_b_reg),

                         .entropy(entropy),

                         .rnd_data(rnd_data),
                         .rnd_valid(rnd_valid),
                         .rnd_ack(rnd_ack),

                         .debug(debug),
                         .debug_update(debug_update)
                        );


  //----------------------------------------------------------------
  // reg_update
  //
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with asynchronous
  // active low reset.
  //----------------------------------------------------------------
  always @ (posedge clk or negedge reset_n)
    begin
      if (!reset_n)
        begin
          en_reg   <= 1;
          op_a_reg <= DEFAULT_OP;
          op_b_reg <= ~DEFAULT_OP;
        end
      else
        begin
          if (en_we)
            begin
              en_reg <= en_new;
            end

          if (op_a_we)
            begin
              op_a_reg <= op_a_new;
            end

          if (op_b_we)
            begin
              op_b_reg <= op_b_new;
            end

         end
    end // reg_update


  //----------------------------------------------------------------
  // api_logic
  //
  // Implementation of the api logic. If cs is enabled will either
  // try to write to or read from the internal registers.
  //----------------------------------------------------------------
  always @*
    begin : api_logic
      en_new        = 0;
      en_we         = 0;
      op_a_new      = 0;
      op_a_we       = 0;
      op_b_new      = 0;
      op_b_we       = 0;
      rnd_ack       = 0;
      tmp_read_data = 32'h00000000;
      tmp_error     = 0;

      if (cs)
        begin
          if (we)
            begin
              case (address)
                // Write operations.
                ADDR_CTRL:
                  begin
                    en_new = write_data[CTRL_ENABLE_BIT];
                    en_we  = 1;
                  end

                ADDR_OPA:
                  begin
                    op_a_new = write_data;
                    op_a_we  = 1;
                  end

                ADDR_OPB:
                  begin
                    op_b_new = write_data;
                    op_b_we  = 1;
                  end

                default:
                  begin
                    tmp_error = 1;
                  end
              endcase // case (address)
            end
          else
            begin
              case (address)
                ADDR_CTRL:
                  begin
                    tmp_read_data[CTRL_ENABLE_BIT] = en_reg;
                  end

                ADDR_STATUS:
                  begin
                    tmp_read_data[STATUS_RND_VALID_BIT] = rnd_valid;
                  end

              ADDR_OPA:
                begin
                  tmp_read_data = op_a_reg;
                end

              ADDR_OPB:
                begin
                  tmp_read_data = op_b_reg;
                end

                ADDR_ENTROPY:
                  begin
                    tmp_read_data = entropy;
                  end

                ADDR_RND:
                  begin
                    tmp_read_data = rnd_data;
                    rnd_ack       = 1;
                  end

                default:
                  begin
                    tmp_error = 1;
                  end
              endcase // case (address)
            end
        end
    end

endmodule // rosc_entropy_core

//======================================================================
// EOF rosc_entropy_core.v
//======================================================================
