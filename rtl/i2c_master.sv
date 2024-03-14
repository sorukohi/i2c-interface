`timescale 1ns / 1ps

module i2c_master #(
  parameter CLK_KHZ = 100_000,
  parameter SCL_KHZ = 10_000
) (
  input  logic       clk_i,
  input  logic       arst_i,
  
  inout  logic       sda_io,
  output logic       scl_o,
  
  input  logic [7:0] data_i,
  output logic [7:0] data_o, 
  
  input  logic [6:0] slave_addr_i,
  input  logic [7:0] reg_addr_i,

  input  logic       slave_addr_zero_bit,
  input  logic       start_trnsc_i,
  input  logic       read_nwrite_i,
  input  logic       nack_i,
  output logic       valid_byte_o,   
  output logic       ready_o
);
  // v------------------ To generate start strobe ------------------v //
  logic strb_start;
  
  strobe_generator #(
    .CLK_KHZ ( CLK_KHZ )
  ) strb_gen (
    .clk_i  ( clk_i        ),
    .arst_i ( arst_i       ),
    .en_i   ( start_trnsc_i),
    .sync_o ( strb_start   )
  );
  // ^--------------------------------------------------------------^ //

  // v------------------ TO GENERATE SCL AND INFO ABOUT ONE ------------------v //
  localparam SCL_FREQ        = SCL_KHZ * 1000;
  localparam CLK_FREQ        = CLK_KHZ * 1000;
  localparam PERIOD_SCL      = CLK_FREQ / SCL_FREQ;
  localparam HALF_PERIOD_SCL = PERIOD_SCL / 2;
  localparam w_cnt_scl       = $clog2(HALF_PERIOD_SCL);
  
  logic [w_cnt_scl : 0] cnt_scl;
  logic                 cnt_scl_increasing; 

  always_ff @(posedge clk_i or posedge arst_i) begin
    if (arst_i) begin
      cnt_scl_increasing <= 'd1; 
      cnt_scl            <= 'd0;    
    end else if (state == IDLE) begin
      cnt_scl_increasing <= 'd1; 
      cnt_scl            <= 'd0; 
    end else begin
      if (cnt_scl_increasing) if (cnt_scl == HALF_PERIOD_SCL - 1) cnt_scl_increasing <= 'd0;
                              else                                cnt_scl            <= cnt_scl + 'd1;
      else                    if (cnt_scl == 'd0)                 cnt_scl_increasing <= 'd1;
                              else                                cnt_scl            <= cnt_scl - 'd1;
    end
  end
  
  logic scl_negedge;
  logic scl_posedge;

  assign scl_negedge = (state != IDLE) && scl_o && (cnt_scl == HALF_PERIOD_SCL - 1);
  assign scl_posedge = (state != IDLE) && scl_o && (cnt_scl == 'd0);

  logic scl_is_low;
  logic scl_is_high;

  assign scl_is_low  = ~scl_o; 
  assign scl_is_high =  scl_o; 
  
  assign scl_o = cnt_scl_increasing;
  // ^------------------------------------------------------------------------^ //

  // v------------------ FSM. TO DEFINE A I2C WORKING STATE ------------------v //
  typedef enum logic [3:0] {

    IDLE,
    START_BIT,
    ADDR,
    W_BIT,
    SLAVE_ACK_TASK,
    REG_ADDR,
    SLAVE_ACK_REG,

    WRITE,
    SLAVE_ACK_W_OP,

    INTRMDT_START_BIT,
    ADDR_REPEAT,
    R_BIT,
    SLAVE_ACK_R_OP,
    READ,
    MASTER_ACK,

    STOP_BIT
    
  } statement_t;
  statement_t state, nextstate;
  
  logic  ack;
  logic  nack;
  logic  r_bit;
  logic  w_bit;
  logic  r_or_w;
  logic sda_state;
  
  assign ack    = 'd0;
  assign w_bit  = 'd0;
  assign r_or_w = read_nwrite_i;
  assign sda_io = sda_state;

  always_ff @(posedge clk_i or posedge arst_i) begin
    if      (arst_i)                                   state <= IDLE;
    else if ((strb_start && ready_o) || state != IDLE) state <= nextstate;
  end

  always_comb begin
    nextstate = state;
    case (state)
      // -----------------------------------------------------------------------------------------------------------------------------------
      // Initial actions: address slave -> register address -> ...

      IDLE              : if      (strb_start)                                  nextstate = START_BIT;   

      START_BIT         : if      (scl_negedge)                                 nextstate = ADDR;

      ADDR              : if      (scl_negedge && cnt_slave_addr == 6)          nextstate = W_BIT;

      W_BIT             : if      (scl_negedge && sda_state != w_bit)           nextstate = STOP_BIT;
                          else if (scl_negedge && sda_state == w_bit)           nextstate = SLAVE_ACK_TASK;

      SLAVE_ACK_TASK    : if      (scl_negedge && (nack_i || sda_io != ack))    nextstate = STOP_BIT;
                          else if (scl_negedge && sda_io == ack && !nack_i)     nextstate = REG_ADDR;

      REG_ADDR          : if      (scl_negedge && cnt_reg_addr == 7)            nextstate = SLAVE_ACK_REG;

      SLAVE_ACK_REG     : if      (scl_negedge && (nack_i || sda_io != ack))    nextstate = STOP_BIT;
                          else if (scl_negedge && sda_io == ack &&  r_or_w)     nextstate = INTRMDT_START_BIT;
                          else if (scl_negedge && sda_io == ack && !r_or_w)     nextstate = WRITE;

      // -----------------------------------------------------------------------------------------------------------------------------------
      //                                                       ... write data into slave -> ack from slave

      WRITE             : if      (scl_negedge && cnt_master_bits == 7)         nextstate = SLAVE_ACK_W_OP;

      SLAVE_ACK_W_OP    : if      (scl_negedge && (nack_i || sda_io != ack))    nextstate = STOP_BIT;
                          else if (scl_negedge && sda_io == ack && !nack_i)     nextstate = WRITE;

      // -----------------------------------------------------------------------------------------------------------------------------------
      //                                                       ... once start bit -> addr slave -> read data from slave -> ack from master

      INTRMDT_START_BIT : if      (scl_negedge)                                 nextstate = ADDR_REPEAT;

      ADDR_REPEAT       : if      (scl_negedge && cnt_slave_addr == 6)          nextstate = R_BIT; 

      R_BIT             : if      (scl_negedge) if (sda_io == w_bit)           nextstate = STOP_BIT;
                                                else                           nextstate = SLAVE_ACK_R_OP;     

      SLAVE_ACK_R_OP    : if      (scl_negedge && (nack_i || sda_io != ack))    nextstate = STOP_BIT;
                          else if (scl_negedge && sda_io == ack && !nack_i)     nextstate = READ;
      
      READ              : if      (scl_negedge && cnt_slave_bits == 7)          nextstate = MASTER_ACK;
      
      MASTER_ACK        : if      (scl_negedge && sda_state !== 'b0)            nextstate = STOP_BIT;
                          else if (scl_negedge)                                 nextstate = READ;

      // -----------------------------------------------------------------------------------------------------------------------------------
        
      STOP_BIT          : if      (scl_is_high)                                 nextstate = IDLE;
      
      // -----------------------------------------------------------------------------------------------------------------------------------
      
       default          :                                                       nextstate = IDLE;
      
      // -----------------------------------------------------------------------------------------------------------------------------------
    endcase
  end
  // ^------------------------------------------------------------------------^ //

  // v------------------ TO SET VALUE ON SDA LINE ------------------v //
  always_comb begin
    case (state)
      // -----------------------------------------------------------------------------------------------------------------------------------
      // Initial actions: address slave -> register address -> ...

      IDLE              :                                                 sda_state <= 'bz;

      START_BIT         : if      (scl_is_high)                           sda_state <= 'b0;

      ADDR              : if      (scl_is_low) if (slave_addr[6] != 'd0)  sda_state <= 'bz;
                                               else                       sda_state <= 'b0;

      W_BIT             : if      (scl_is_low)                            sda_state <= 'b0;

      SLAVE_ACK_TASK    : if      (scl_is_low)                            sda_state <= 'bz;

      REG_ADDR          : if      (scl_is_low) if (reg_addr[7] != 'd0)    sda_state <= 'bz;
                                               else                       sda_state <= 'b0;

      SLAVE_ACK_REG     : if      (scl_is_low)                            sda_state <= 'bz;

      // -----------------------------------------------------------------------------------------------------------------------------------
      //                                                       ... write data into slave -> ack from slave

      WRITE             : if      (scl_is_low) if (master_bits[7] != 'd0) sda_state <= 'bz;
                                               else                       sda_state <= 'b0;

      SLAVE_ACK_W_OP    : if      (scl_is_low)                            sda_state <= 'bz;

      // -----------------------------------------------------------------------------------------------------------------------------------
      //                                                       ... once start bit -> addr slave -> read data from slave -> ack from master

      INTRMDT_START_BIT : if      (scl_is_high)                           sda_state <= 'b0;

      ADDR_REPEAT       : if      (scl_is_low) if (slave_addr[6] != 'd0)  sda_state <= 'bz;
                                               else                       sda_state <= 'b0;

      R_BIT             : if      (scl_is_low)                            sda_state <= 'bz;

      SLAVE_ACK_R_OP    : if      (scl_is_low)                            sda_state <= 'bz;
      
      READ              : if      (scl_is_low)                            sda_state <= 'bz;
      
      MASTER_ACK        : if      (scl_is_low) if (nack_i)                sda_state <= 'bz;
                                               else                       sda_state <= 'b0;

      // -----------------------------------------------------------------------------------------------------------------------------------
        
      STOP_BIT          : if      (scl_is_low)                            sda_state <= 'b0;
                          else if (scl_is_high)                           sda_state <= 'bz;
      
      // -----------------------------------------------------------------------------------------------------------------------------------
      
       default          :                                                 sda_state <= 'bz;
      
      // -----------------------------------------------------------------------------------------------------------------------------------
    endcase
  end
  // ^-------------------------------------------------------------^ //
  
  // v------------------ TO SEND SLAVE ADDRESS ------------------v //
  logic [6:0] slave_addr;
  
  always_ff @(posedge clk_i or posedge arst_i) begin
    if      (arst_i)                                slave_addr <= slave_addr_i;
    else if (state != ADDR && state != ADDR_REPEAT) slave_addr <= slave_addr_i;
    else if (scl_negedge)                           slave_addr <= {slave_addr[5:0], slave_addr[6]}; // left shift 
  end
  
  logic [$clog2(7) : 0] cnt_slave_addr;
  
  always_ff @(posedge clk_i or posedge arst_i) begin
    if      (arst_i)                                cnt_slave_addr <= 'd0;
    else if (state != ADDR && state != ADDR_REPEAT) cnt_slave_addr <= 'd0;
    else if (scl_negedge)                           cnt_slave_addr <= cnt_slave_addr + 'd1;  
  end
  // ^-----------------------------------------------------------^ //

  // v------------------ TO SEND REG ADDRESS ------------------v //
  logic [7:0] reg_addr;
  
  always_ff @(posedge clk_i or posedge arst_i) begin
    if      (arst_i)            reg_addr <= reg_addr_i;
    else if (state != REG_ADDR) reg_addr <= reg_addr_i;
    else if (scl_negedge)       reg_addr <= {reg_addr[6:0], reg_addr[7]}; // left shift 
  end
  
  logic [$clog2(8) : 0] cnt_reg_addr;
  
  always_ff @(posedge clk_i or posedge arst_i) begin
    if      (arst_i)            cnt_reg_addr <= 'd0;
    else if (state != REG_ADDR) cnt_reg_addr <= 'd0;
    else if (scl_negedge)       cnt_reg_addr <= cnt_reg_addr + 'd1;  
  end
  // ^---------------------------------------------------------^ //

  // v------------------ TO WRITE DATA INTO SLAVE ------------------v //
  logic [7:0] master_bits;
  
  always_ff @(posedge clk_i or posedge arst_i) begin
    if      (arst_i)          master_bits <= data_i;
    else if (state != WRITE) master_bits <= data_i;
    else if (scl_negedge)    master_bits <= {master_bits[6:0], master_bits[7]}; // left shift 
  end

  logic [$clog2(8) : 0] cnt_master_bits;
  
  always_ff @(posedge clk_i or posedge arst_i) begin
    if      (arst_i)          cnt_master_bits <= 'd0;
    else if (state != WRITE) cnt_master_bits <= 'd0;
    else if (scl_negedge)    cnt_master_bits <= cnt_master_bits + 'd1;  
  end
  // ^--------------------------------------------------------------^ //

  // v------------------ TO READ SLAVE DATA ------------------v //
  logic [7:0] slave_bits;
  
  always_ff @(posedge clk_i or posedge arst_i) begin
    if      (arst_i)                               slave_bits <= '1;
    else if (state != READ && state != MASTER_ACK)                   slave_bits <= '1;
    else if (scl_negedge) if (sda_io == 1'd0) slave_bits <= {slave_bits[6:0], 1'b0}; // left shift 
                          else               slave_bits <= {slave_bits[6:0], 1'b1};
  end

  logic [$clog2(8) : 0] cnt_slave_bits;
  
  always_ff @(posedge clk_i or posedge arst_i) begin
    if      (arst_i)        cnt_slave_bits <= 'd0;
    else if (state != READ) cnt_slave_bits <= 'd0;
    else if (scl_negedge)   cnt_slave_bits <= cnt_slave_bits + 'd1;  
  end
  // ^--------------------------------------------------------^ //

  assign data_o       = slave_bits;
  assign valid_byte_o = (state == MASTER_ACK) && (scl_is_high);
  assign ready_o      = (state == IDLE); 
  
endmodule