//======================================================================
//
// rosc.v
// ---------
// Digital ring oscillator used as entropy source.
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

module rosc(
            input wire clk,
            input wire we,
            input wire reset_n,

            input [(WIDTH - 1) : 0] op_a,
            input [(WIDTH - 1) : 0] op_b,
            output wire             d
           );

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter l=2;

  //----------------------------------------------------------------
  //----------------------------------------------------------------
   wire 	 cin;
   wire [l:0]  sum = in1 + in2 + cin;


  //----------------------------------------------------------------
  //----------------------------------------------------------------
   assign cin = ~sum[l];


  //----------------------------------------------------------------
  //----------------------------------------------------------------
   always @(posedge clk or negedge reset_n)
     if(!nreset)
       begin
         d <= 0;
       end
     else
       begin
         if (we)
           begin
             d <= sum[l];
           end
       end

endmodule // rosc

//======================================================================
// EOF rosc.v
//======================================================================
