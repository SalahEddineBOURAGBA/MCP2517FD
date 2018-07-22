#pragma once 

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#include "CANFDFrame.h"
#include <math.h>
#include <SPI.h>

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

class MCP2517FD {

  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  // Class members
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  
  //--- New defined types
  public: typedef enum {QUARTZ_4MHz, QUARTZ_20MHz, QUARTZ_40MHz} QuartzEnum;
  public: typedef enum {NormalMode, SleepMode, InternalLoopbackMode, ListenOnlyMode, ConfigurationMode, 
                        ExternalLoopbackMode, CANMode} ConfMode;
  private: typedef struct {
          uint32_t mBitRate;
          uint8_t  mBTQ;
          uint8_t  mPrescaler;
          uint8_t  mPropSeg;
          uint8_t  mPhaseSeg1;
          uint8_t  mPhaseSeg2;
          uint8_t  mSJW;
          uint32_t mSamplePoint;
         } bitTime;

  //--- Private members
  private: SPISettings mSPISettings ;
  private: SPIClass & mSPI ;
  private: byte mCS ;
  private: QuartzEnum mQuartz ;
  private: bitTime nominalBitTime, dataBitTime;
  private: uint32_t mBusLength;
  private: uint32_t mTRCV;
  private: uint32_t mCANFDTolerance;


  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  // Constructors
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

  //--- Default constructor
  /*
   * Nominal bitTime = 500 Kb/s
   * Data bitTime = 1 Mb/s
   * BusLength = 40 m, TRCV = 80 ns
   * CANTolerance = 30 PPM
   */
  public: MCP2517FD (SPIClass & inSPI, const byte inCS, const QuartzEnum inQuartz);

  
  //--- Constructor with bitRates
  /*
   * BusLength = 40 m, TRCV = 80 ns
   * CANTolerance = 30 PPM
   */
  public: MCP2517FD (SPIClass & inSPI, const byte inCS, const QuartzEnum inQuartz, const uint32_t inNominalBitRate,
                     const uint32_t inDataBitRate);

  
  //--- Constructor with BiteRates, samplePoint and bus Properties                     
  public: MCP2517FD (SPIClass & inSPI, const byte inCS, const QuartzEnum inQuartz, const uint32_t inNominalBitRate,
                     const uint32_t inDataBitRate, const uint32_t inNominalSamplePoint, const uint32_t inDataSamplePoint, 
                     const uint32_t inBusLength, const uint32_t inTRCV, const uint32_t inCANFDTolerance);


  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  // Configure SPI and bitTime
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

  //--- Initilize SPI connection, MCP2715FD registers and bitTime
  public: uint32_t begin (void);
  // |
  // |
  // -> //--- Reset MCP2517FD registers
        private: void reset2517FD (void);
  // |
  // |
  // -> //--- Configure bitTime
        private: uint32_t bitTimeSettings (void);

  
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  // Configure TEF
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

  //--- Enable or disable TEF, and configure FIFO's size if enabled
  /*
   * inState = 1 --> enable TEF
   * inFIFOSize in [1,32]
   * inTimeStamp = 1 --> enable message stamping
   */
  public: bool configureTEF (const bool inState = 0, const uint32_t inFIFOSize = 1, const bool inTimeStamp = 0);

  //--- Get TEF FIFO state
  /*
   * TEF FIFO not full --> return 1
   * TEF FIFO full     --> return 0
   */
  public: bool isTEFFIFONotFull (void);

  //--- Reset TEF
  public: bool resetTEF (void);


  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  // Configure TXQ
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

  //--- Enable or disable TEF, and configure FIFO's size if enabled
  /*
   * inState = 1 --> enable TXQ
   * inFIFOSize in [1,32]
   * inPayloadSize --> Data Size
   * inPriority in [0,100] --> priority to the highest value
   */
  public: bool configureTXQ (const bool inState = 1, const uint32_t inFIFOSize = 10, const uint32_t inPayloadSize = 16,
                             const uint32_t inPriority = 100);

  //--- Get TXQ FIFO state
  /*
   * TXQ FIFO not full --> return 1
   * TXQ FIFO full     --> return 0
   */
  public: bool isTXQFIFONotFull (void);

  //--- Configure retransmission Attempts
  /*
   * 0 --> disable retransmission
   * 1 --> 3 retransmission attempts
   * else for unlimited retransmission attempts
   */
  public: void changeTXQRetransmissionAttempts (const uint32_t inRetransmissionAttempts);                             
  
  //--- reset TXQ
  public: bool resetTXQ (void);


  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  // Configure transmit and receive FIFOs
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

  //--- Configure FIFOs
  /*
   * m in [1,32] --> FIFOm
   * inState = 1 --> Transmit FIFO, inState = 0 --> Receive FIFO
   * inFIFOSize in [1,32]
   * inPayloadSize --> Data Size
   * inPriority in [0,100] --> priority to the highest value
   */
  public: bool configureFIFOm (const uint16_t m = 1, const bool inState = 1, const uint32_t inFIFOSize = 10, 
                               const uint32_t inPayloadSize = 16, const uint32_t inPriority = 0, const bool inTimeStamp = 0);

  //--- Get FIFOm state
  /*
   * Transmit FIFO --> return 1
   * Receive FIFO  --> return 0
   */
  public: bool isFIFOmTXorRX (const uint16_t m);
  
  //--- Get Transmit FIFOm's FIFO state
  /*
   * TXQ FIFO not full --> return 1
   * TXQ FIFO full     --> return 0
   */
  public: bool isTransmitFIFOmNotFull (const uint16_t m);

  //--- Get Receive FIFOm's FIFO state
  /*
   * TXQ FIFO not empty --> return 1
   * TXQ FIFO empty     --> return 0
   */
  public: bool isReceiveFIFOmNotEmpty (const uint16_t m);

  //--- Configure FIFOm retransmission Attempts
  /*
   * 0 --> disable retransmission
   * 1 --> 3 retransmission attempts
   * else for unlimited retransmission attempts
   */
  public: bool changeFIFOmRetransmissionAttempts (const uint16_t m, const uint32_t inRetransmissionAttempts);

  //--- reset FIFOm
  public: bool resetFIFOm (const uint16_t m);
  
  
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  // Configure filters
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

  //--- Configure receive filter (at least one must be configured)
  /*
   * m in [1,32] --> 32 filters
   * inFIFO in [1,32] --> save messages that pass the filter to receive FIFOm
   * inMatchExtAndStandId = 0 --> match both standard and extended ID, = 1 --> match one of them
   * inMatchExtOrStandId --> if match one of the standard and extended ID is selected then
   *   if inMatchExtOrStandId = 0 then match only messages with standard ID
   *   if inMatchExtOrStandId = 1 then match only messages with extended ID
   * inStandIdRange[i] = 1 --> instandId[i] must be exact, else it can take any value
   */
  public: bool configureFilterm (const uint16_t m = 1, const uint32_t inFIFO = 1, const bool inMatchExtAndStandId = 1, 
                                 const uint32_t inStandId = 0x000, const uint32_t inStandIdRange = 0x000, 
                                 const uint32_t inExtId = 0x00000, const uint32_t inExtIdRange = 0x00000, 
                                 const bool inMatchExtOrStandId = 0);
  
  //--- Activate filter m (m in [1,32])
  public: bool enableFilterm (const uint16_t m);
  
  //--- Desactivate filter m (m in [1,32])
  public: bool disableFilterm (const uint16_t m);
  
  
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  // Configure operation mode
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

  //--- MCP517FD mode in: {NormalMode, SleepMode, InternalLoopbackMode, ListenOnlyMode, ConfigurationMode, 
  //                      ExternalLoopbackMode, CANMode}
  public: void configureMode (const ConfMode inMode);

  //--- get configurationMode
  public: ConfMode getConfigurationMode (void);

  
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  // Send and receive frames
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

  //--- Load a frame into the transmission channel and request transmission
  /*
   * inChannel = 0 --> TxQ, = m in [1,32] --> FIFOm
   * inDontSend = 0 --> request transmission 
   */  
  public: bool sendFrame (const uint32_t inChannel, const CANFDFrame &inFrame, const bool inDontSend = 0);

  //--- Request transmission of all frames or frames in specific channels
  /*
   * inTransmitMode = 0 --> request transmission of all messages
   * inTransmitMode = 1 --> request transmission of messages in inChannel x (0:TXQ,[1,32]:FIFOm)
   * inTransmitMode = else --> request transmission of messages in channels where inChannel[i]=1
   */
  public: bool requestTransmission (const uint8_t inTransmitMode = 0, const uint32_t inChannel = 0);

  //--- Receive frame stored in FIFOm (m = inChannel in [1,32])
  public: CANFDFrame* receiveFrame (const uint32_t inChannel);

  //--- Abort transmission of stored frames in TXQ or FIFO
  /*
   * Abort only frames that are stored in FIFOs and not in transmission
   * inChannel = 0 --> TxQ, = m in [1,32] --> FIFOm
   */
  public: bool abortTransmission (const uint32_t inChannel);


  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  // Time Stamping
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  
  //--- Example of utilisation
  /*
   * disable time stamping
   * configure prescaler
   * configure time stamping position (default at the beggining, at SOF)
   * enable time stamping
   */

  //--- Enable Time Stamping
  public: bool enableTimeStamping (void);

  //--- Disable, stop and reset the TimeBaseCounter
  public: bool disableTimeStamping (void);

  //--- Configure TimeBaseCounter Prescaler
  /*
   * Time Base Counter increments on Prescaler*SYSCLK
   * Prescaler in [0,1023]
   */
  public: bool configureTimeStampingPrescaler (const uint32_t inPrescaler);

  //--- Configure time stamping position
  /*
   * atBeginning=0 ==> time stamp at the end of the frame after it's valid
   * atBeginning=1 ==> time stamp at the beginning of the frame, if CAN2.0 then at SOF bit, if CANFD then:
   * ---|=> atSOForRES=0 ==> time stamp at SOF bit (default)
   * ---|=> atSOForRES=1 ==> time stamp at RES bit
   */
  public: bool configureTimeStampingPosition (const bool atBeginning, const bool atSOForRES = 0);

  //--- Read current TimeBaseCounter value
  public: uint32_t getTBC (void);


  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  // Private methods
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

  //--- Write data in a MCP2517FD's register
  private: void writeRegister (const uint16_t inAddress, const uint32_t inValue);
  
  
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  // Public methods
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

  //--- Read data from a MCP2517FD's register
  public: uint32_t readRegister (const uint16_t inAddress);

  //--- Return the ceil value of the division of two integers
  public: uint32_t intCeil (const uint32_t a, const uint32_t b);

  
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  // No copy
  //————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

  private: MCP2517FD (const MCP2517FD &);
  private: MCP2517FD& operator = (const MCP2517FD &other);
} ;

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

