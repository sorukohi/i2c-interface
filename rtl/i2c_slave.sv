`timescale 1ns / 1ps

module i2c_slave (
  input  logic       arst_i,
  
  inout  logic       sda_io,
  input  logic       scl_i,
  
  input  logic [7:0] data_i,
  output logic [7:0] data_o,  

  input  logic [6:0] mine_addr_i,
  output logic [7:0] reg_addr_o,

  output logic       valid_reg_o,
  output logic       valid_byte_o
);

  // v------------------ INFO ABOUT SCL ------------------v //
  logic scl_is_low;
  logic scl_is_high;

  assign scl_is_low  = ~scl_i; 
  assign scl_is_high =  scl_i; 
  // ^----------------------------------------------------^ //

  // v------------------ FSM. TO DEFINE A I2C WORKING STATE ------------------v //
  typedef enum logic [4:0] {

    IDLE,
    START_BIT,
    ADDR,
    W_BIT,
    SLAVE_ACK_TASK,
    REG_ADDR,
    SLAVE_ACK_REG,

    DEFINE_OP,

    M_WRITE,
    SLAVE_ACK_W_OP,

    INTRMDT_START_BIT,
    ADDR_REPEAT,
    R_BIT,
    SLAVE_ACK_R_OP,
    M_READ,
    MASTER_ACK,

    STOP_BIT
    
  } statement_t;
  statement_t state, nextstate;
  
  logic ack;
  logic w_bit;
  logic sda_state;
  logic is_start_bit;
  logic is_stop_bit;

  assign ack    = 'd0;
  assign w_bit  = 'd0;
  assign sda_io = sda_state;
  
  always_ff @(negedge sda_io or posedge arst_i) begin
    if      (arst_i)                 is_start_bit <= 'd0;
    else if (!is_start_bit && scl_i) is_start_bit <= 'd1;
    else                             is_start_bit <= 'd0;
  end

  always_ff @(posedge sda_io or posedge arst_i) begin
    if      (arst_i)                is_stop_bit <= 'd0;
    else if (!is_stop_bit && scl_i) is_stop_bit <= 'd1;
    else                            is_stop_bit <= 'd0;
  end

  logic real_stop;

  always_ff @(nextstate) begin
    if      (arst_i)      real_stop <= 'd0;
    else if (is_stop_bit) real_stop <= 'd1;
    else                  real_stop <= 'd0;
  end

  always_ff @(negedge scl_i or posedge arst_i or posedge is_start_bit or posedge real_stop) begin
    if (arst_i) state <= IDLE;
    else        state <= nextstate;
  end

  always_comb begin
    nextstate = state;
    case (state)
      // -----------------------------------------------------------------------------------------------------------------------------------
      // Initial actions: address slave -> register address -> ...

      IDLE              : if      (is_start_bit)                   nextstate = ADDR;   

      START_BIT         :                                          nextstate = ADDR;

      ADDR              : if      (cnt_slave_addr == 6)            nextstate = W_BIT;

      W_BIT             : if      (sda_io != w_bit || is_stop_bit) nextstate = IDLE; 
                          else                                     nextstate = SLAVE_ACK_TASK;

      SLAVE_ACK_TASK    : if      (sda_io != ack || is_stop_bit)   nextstate = IDLE;
                          else                                     nextstate = REG_ADDR;

      REG_ADDR          : if      (cnt_reg_addr == 7)              nextstate = SLAVE_ACK_REG;

      SLAVE_ACK_REG     : if      (sda_io != ack)                  nextstate = IDLE;
                          else                                     nextstate = DEFINE_OP;
    
      DEFINE_OP         : if      (is_stop_bit)                    nextstate = IDLE;
                          else if (is_start_bit)                   nextstate = ADDR_REPEAT;
                          else                                     nextstate = M_WRITE;

      // -----------------------------------------------------------------------------------------------------------------------------------
      //                                                       ... write data into slave -> ack from slave

      M_WRITE           : if      (is_stop_bit)                    nextstate = IDLE;
                          else if (is_start_bit)                   nextstate = ADDR_REPEAT;
                          else if (cnt_master_bits == 7)           nextstate = SLAVE_ACK_W_OP;

      SLAVE_ACK_W_OP    : if      (sda_io != ack || is_stop_bit)   nextstate = IDLE;
                          else                                     nextstate = M_WRITE;

      // -----------------------------------------------------------------------------------------------------------------------------------
      //                                                       ... once start bit -> addr slave -> read data from slave -> ack from master

      ADDR_REPEAT       : if      (cnt_slave_addr == 6)            nextstate = R_BIT; 

      R_BIT             : if      (sda_io == w_bit || is_stop_bit) nextstate = IDLE;
                          else                                     nextstate = SLAVE_ACK_R_OP;       

      SLAVE_ACK_R_OP    : if      (sda_io != ack || is_stop_bit)   nextstate = IDLE;
                          else                                     nextstate = M_READ;
      
      M_READ            : if      (cnt_slave_bits == 7)            nextstate = MASTER_ACK;
      
      MASTER_ACK        : if      (sda_io === 1'bz || is_stop_bit) nextstate = STOP_BIT;
                          else if (is_start_bit)                   nextstate = ADDR; 
                          else                                     nextstate = M_READ;

      // -----------------------------------------------------------------------------------------------------------------------------------
        
      STOP_BIT          : if      (scl_is_high)                    nextstate = IDLE;
      
      // -----------------------------------------------------------------------------------------------------------------------------------
      
      default           :                                          nextstate = IDLE;
      
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

      START_BIT         :                                                 sda_state <= 'dz;

      ADDR              :                                                 sda_state <= 'dz;

      W_BIT             :                                                 sda_state <= 'dz;

      SLAVE_ACK_TASK    : if (sda_io != 'd0 || slave_addr != mine_addr_i) sda_state <= 'b0;
                          else                                            sda_state <= 'bz;

      REG_ADDR          :                                                 sda_state <= 'bz;

      SLAVE_ACK_REG     :                                                 sda_state <= 'b0;
                            

      // -----------------------------------------------------------------------------------------------------------------------------------
      //                                                       ... write data into slave -> ack from slave

      M_WRITE           :                                                 sda_state <= 'bz;

      SLAVE_ACK_W_OP    :                                                 sda_state <= 'b0;

      // -----------------------------------------------------------------------------------------------------------------------------------
      //                                                       ... once start bit -> addr slave -> read data from slave -> ack from master

      INTRMDT_START_BIT :                                                 sda_state <= 'b0;

      ADDR_REPEAT       :                                                 sda_state <= 'bz;
                                    
      R_BIT             :                                                 sda_state <= 'bz;

      SLAVE_ACK_R_OP    : if (sda_io == 'd0 || slave_addr != mine_addr_i) sda_state <= 'b0;
                          else                                            sda_state <= 'bz;
      
      M_READ            : if (slave_bits[7] != 'd0)                       sda_state <= 'bz;
                          else                                            sda_state <= 'b0;
      
      MASTER_ACK        :                                                 sda_state <= 'bz;

      // -----------------------------------------------------------------------------------------------------------------------------------
        
      STOP_BIT          :                                                 sda_state <= 'bz;
      
      // -----------------------------------------------------------------------------------------------------------------------------------
      
      default           :                                                 sda_state <= 'bz;
      
      // -----------------------------------------------------------------------------------------------------------------------------------
    endcase
  end
  // ^-------------------------------------------------------------^ //
  
  // v------------------ TO RECIEVE DVC ADDRESS ------------------v //
  logic [6:0] slave_addr;
  
  always_ff @(negedge scl_i or posedge arst_i) begin
    if      (arst_i)                                                      slave_addr <= 'd0;
    else if (state != ADDR && state != ADDR_REPEAT && state != DEFINE_OP) slave_addr <= 'd0;
    else if (state == W_BIT)                                              slave_addr <= slave_addr;
    else if (sda_io != 'd0)                                               slave_addr <= {slave_addr[5:0], 1'b1}; // left shift
    else                                                                  slave_addr <= {slave_addr[5:0], 1'b0}; 
  end
  
  logic [$clog2(7) : 0] cnt_slave_addr;
  
  always_ff @(negedge scl_i or posedge arst_i) begin
    if      (arst_i)                                                      cnt_slave_addr <= 'd0;
    else if (state != ADDR && state != ADDR_REPEAT && state != DEFINE_OP) cnt_slave_addr <= 'd0;
    else                                                                  cnt_slave_addr <= cnt_slave_addr + 'd1;  
  end
  // ^-----------------------------------------------------------^ //

  // v------------------ TO RECEIVE REG ADDRESS ------------------v //
  logic [7:0] reg_addr;
  
  always_ff @(negedge scl_i or posedge arst_i) begin
    if      (arst_i)            reg_addr <= 'd0;
    else if (state != REG_ADDR && state != SLAVE_ACK_TASK) reg_addr <= 'd0;
    else if (sda_io !== 'd0)     reg_addr <= {reg_addr[6:0], 1'b1}; // left shift
    else                        reg_addr <= {reg_addr[6:0], 1'b0}; 
  end
  
  logic [$clog2(8) : 0] cnt_reg_addr;
  
  always_ff @(negedge scl_i or posedge arst_i) begin
    if      (arst_i)            cnt_reg_addr <= 'd0;
    else if (state != REG_ADDR) cnt_reg_addr <= 'd0;
    else                        cnt_reg_addr <= cnt_reg_addr + 'd1;  
  end
  // ^------------------------------------------------------------^ //

  // v------------------ TO READ DATA FROM MASTER ------------------v //
  logic [7:0] master_bits;
  
  always_ff @(negedge scl_i or posedge arst_i) begin
    if      (arst_i)                                 master_bits <= 'd0;
    else if (state != M_WRITE && state != DEFINE_OP) master_bits <= 'd0;
    else if (sda_io === 1'bz)                        master_bits <= {master_bits[6:0], 1'b1}; // left shift
    else                                             master_bits <= {master_bits[6:0], 1'b0}; 
  end

  logic [$clog2(8) : 0] cnt_master_bits;
  
  always_ff @(negedge scl_i or posedge arst_i) begin
    if      (arst_i)                                 cnt_master_bits <= 'd0;
    else if (state != M_WRITE && state != DEFINE_OP) cnt_master_bits <= 'd0;
    else                                             cnt_master_bits <= cnt_master_bits + 'd1;  
  end
  // ^--------------------------------------------------------------^ //

  // v------------------ TO WRITE DATA TO MASTER ------------------v //
  logic [7:0] slave_bits;
  
  always_ff @(negedge scl_i or posedge arst_i) begin
    if      (arst_i)          slave_bits <= data_i;
    else if (state != M_READ && cnt_slave_bits <= 'd8) slave_bits <= data_i;
    else                      slave_bits <= {slave_bits[6:0], slave_bits[7]}; // right shift 
  end

  logic [$clog2(8) : 0] cnt_slave_bits;
  
  always_ff @(negedge scl_i or posedge arst_i) begin
    if      (arst_i)          cnt_slave_bits <= 'd0;
    else if (state != M_READ) cnt_slave_bits <= 'd0;
    else                      cnt_slave_bits <= cnt_slave_bits + 'd1;  
  end
  // ^-------------------------------------------------------------^ //

  assign data_o       = master_bits;
  assign valid_reg_o  = (state == SLAVE_ACK_REG)  && (scl_is_high);
  assign valid_byte_o = (state == SLAVE_ACK_W_OP) && (scl_is_high);
  assign reg_addr_o   = reg_addr;
  
endmodule