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
  parameter ADDR_CTRL            = 8'h10;
  parameter CTRL_ENABLE_BIT      = 0;

  parameter ADDR_STATUS          = 8'h01;
  parameter STATUS_RND_VALUD_BIT = 0;

  parameter ADDR_OPA             = 8'h08;
  parameter ADDR_OPB             = 8'h09;

  parameter ADDR_RND             = 8'h10;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg rnd_ack;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------


  //----------------------------------------------------------------
  // module instantiations.
  //----------------------------------------------------------------

  rosc_entropy_core core(
                         input wire           clk,
                         input wire           reset_n,

                         input wire           enable,

                         input wire [31 : 0]  opa,
                         input wire [31 : 0]  opb,

                         output wire [31 : 0] rnd_data,
                         output wire          rnd_valid,
                         input wire           rnd_ack,

                         output wire [7 : 0]  debug,
                         input wire           debug_update
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

        end
      else
        begin

         end
    end // reg_update

endmodule // rosc_entropy_core

//======================================================================
// EOF rosc_entropy_core.v
//======================================================================
