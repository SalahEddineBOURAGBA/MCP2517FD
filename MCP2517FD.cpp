#include "MCP2517FD.h"

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// MCP2517FD register addresses
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

static const uint16_t OSC_REGISTER        = 0xE00;
static const uint16_t C1CON_REGISTER      = 0x000;
static const uint16_t C1TXREQ_REGISTER    = 0x030;

static const uint16_t C1NBTCFG_REGISTER   = 0x004;
static const uint16_t C1DBTCFG_REGISTER   = 0x008;

static const uint16_t C1TEFCON_REGISTER   = 0x040;
static const uint16_t C1TEFSTA_REGISTER   = 0x044;
static const uint16_t C1TEFUA_REGISTER    = 0x048;

static const uint16_t C1TXQCON_REGISTER   = 0x050;
static const uint16_t C1TXQSTA_REGISTER   = 0x054;
static const uint16_t C1TXQUA_REGISTER    = 0x058;

static const uint16_t C1FIFOCON1_REGISTER = 0x05C;
static const uint16_t C1FIFOSTA1_REGISTER = 0x060;
static const uint16_t C1FIFOUA1_REGISTER  = 0x064;

static const uint16_t C1FLTCON0_REGISTER  = 0x1D0;
static const uint16_t C1FLTOBJ0_REGISTER  = 0x1F0;
static const uint16_t C1MASK0_REGISTER    = 0x1F4;

static const uint16_t C1TSCON_REGISTER    = 0x014;
static const uint16_t C1TBC_REGISTER      = 0x010;



//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// Constructors
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

MCP2517FD::MCP2517FD (SPIClass & inSPI, const byte inCS, const QuartzEnum inQuartz):
  mSPISettings(),
  mSPI(inSPI)
{
  mCS                         = inCS;
  mQuartz                     = inQuartz;
  nominalBitTime.mBitRate     = 500 * 1000;
  nominalBitTime.mSamplePoint = 0;
  dataBitTime.mBitRate        = 1 * 1000 * 1000;
  dataBitTime.mSamplePoint    = 0;
  mBusLength                  = 40;
  mTRCV                       = 80;
  mCANFDTolerance             = 30;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

MCP2517FD::MCP2517FD (SPIClass & inSPI, const byte inCS, const QuartzEnum inQuartz, const uint32_t inNominalBitRate,
                      const uint32_t inDataBitRate):
  mSPISettings(),
  mSPI(inSPI)
{
  mCS                         = inCS;
  mQuartz                     = inQuartz;
  nominalBitTime.mBitRate     = inNominalBitRate;
  nominalBitTime.mSamplePoint = 0;
  dataBitTime.mBitRate        = inDataBitRate;
  dataBitTime.mSamplePoint    = 0;
  mBusLength                  = 40;
  mTRCV                       = 80;
  mCANFDTolerance             = 30;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

MCP2517FD::MCP2517FD (SPIClass & inSPI, const byte inCS, const QuartzEnum inQuartz, const uint32_t inNominalBitRate,
                      const uint32_t inDataBitRate, const uint32_t inNominalSamplePoint, const uint32_t inDataSamplePoint,
                      const uint32_t inBusLength, const uint32_t inTRCV, const uint32_t inCANFDTolerance):
  mSPISettings(),
  mSPI(inSPI)
{
  mCS                         = inCS;
  mQuartz                     = inQuartz;
  nominalBitTime.mBitRate     = inNominalBitRate;
  nominalBitTime.mSamplePoint = inNominalSamplePoint;
  dataBitTime.mBitRate        = inDataBitRate;
  dataBitTime.mSamplePoint    = inDataSamplePoint;
  mBusLength                  = inBusLength;
  mTRCV                       = inTRCV;
  mCANFDTolerance             = inCANFDTolerance;
}



//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// Configure SPI and bitTime
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint32_t MCP2517FD::begin(void)
{
  uint32_t result = 0 ; // Means no error
  pinMode (mCS, OUTPUT) ;
  digitalWrite (mCS, HIGH) ;

  //-------------------------------------------------------------------------
  // Step :1 Reset MCP2517FD registers
  //-------------------------------------------------------------------------

  //--- Set SPI clock to 1 MHz
  mSPISettings = SPISettings (1 * 1000 * 1000, MSBFIRST, SPI_MODE0) ;

  //--- Reset MCP2517FD registers (always use a 1 MHz clock)
  reset2517FD () ;

  //--- Check if SPI connection is on (with a 1 MHz clock)
  // We write and then read back from the MCP2517FD's RAM at address 0x400
  for (uint32_t i = 1 ; (i != 0) && (result == 0) ; i <<= 1)
  {
    writeRegister (0x400, i) ;
    const uint32_t readBackValue = readRegister (0x400) ;
    if (readBackValue != i)
    {
      return 0x100 ; // Error "read back error with a 1 MHz SPI clock"
    }
  }


  //-------------------------------------------------------------------------
  // Step 2: Configure oscillator
  //-------------------------------------------------------------------------

  //--- Now, set internal clock to 20 or 40 MHz
  switch (mQuartz)
  {
    case QUARTZ_4MHz : // Enable x10 PLL -> SYSCLK = 40 MHz
      writeRegister (OSC_REGISTER, 1) ; // PLLEN is 1
      //--- We should use the 20 MHz SPI clock, but SPI max clock frequency is 12 MHz on Adafruit-feather-M0
      mSPISettings = SPISettings (12 * 1000 * 1000, MSBFIRST, SPI_MODE0) ;
      break ;
    case QUARTZ_20MHz : // Do nothing -> SYSCLK = 20 MHz
      //--- We use the 10 MHz SPI clock
      mSPISettings = SPISettings (10 * 1000 * 1000, MSBFIRST, SPI_MODE0) ;
      break ;
    case QUARTZ_40MHz : // Do nothing -> SYSCLK = 40 MHz
      //--- We should use the 20 MHz SPI clock, but SPI max clock frequency is 12 MHz on Adafruit-feather-M0
      mSPISettings = SPISettings (12 * 1000 * 1000, MSBFIRST, SPI_MODE0) ;
      break ;
  }

  //--- Check if SPI connection is on (with a 20 or 40 MHz clock)
  // We write and the read back from the MCP2517FD's RAM at address 0x400
  for (uint32_t i = 1 ; (i != 0) && (result == 0) ; i <<= 1)
  {
    writeRegister (0x400, i) ;
    const uint32_t readBackValue = readRegister (0x400) ;
    if (readBackValue != i)
    {
      return 0x200 ; // Error "read back error with a full-speed SPI clock"
    }
  }


  //-------------------------------------------------------------------------
  // Step 3: Configure bitTime
  //-------------------------------------------------------------------------

  //--- Configure bit time registers
  result = bitTimeSettings();


  return result ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void MCP2517FD::reset2517FD (void)
{
  digitalWrite (mCS, LOW) ;
  mSPI.beginTransaction (mSPISettings) ; // Check RESET is performed with 1 MHz clock
  mSPI.transfer16 (0x00) ; // Reset instruction: 0x0000
  mSPI.endTransaction () ;
  digitalWrite (mCS, HIGH) ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint32_t MCP2517FD::bitTimeSettings (void)
{
  uint32_t bitSettingsOk = 0; //for errors

  uint32_t nominalPhaseSeg1; //1..16
  uint32_t nominalPhaseSeg2; //2..16
  uint32_t nominalPropSeg; //1..48
  uint32_t nominalSJW; //1..16
  uint32_t nominalSamplePoint;
  uint32_t NPS2min;
  uint32_t NPS2max;

  uint32_t dataPhaseSeg1; //1..8
  uint32_t dataPhaseSeg2; //2..8
  uint32_t dataPropSeg = 0; //0..8
  uint32_t dataSJW; //1..8
  uint32_t dataSamplePoint;
  uint32_t DPS2min;
  uint32_t DPS2max;

  uint32_t CANFDFrequency;
  switch (mQuartz)
  {
    case QUARTZ_4MHz :
      CANFDFrequency = 40 * 1000 * 1000;
      break ;
    case QUARTZ_20MHz :
      CANFDFrequency = 20 * 1000 * 1000;
      break ;
    case QUARTZ_40MHz :
      CANFDFrequency = 40 * 1000 * 1000;
      break ;
  }

  //---------------------------------------------------------------------
  // Nominal configuration
  //---------------------------------------------------------------------

  //Step 1: Find nominalPrescaler and nominalBTQ

  uint32_t nominalBTQ     = 80; //8..80
  uint32_t tempNominalBTQ = 80; //start from max then decrement
  uint32_t bestError = UINT32_MAX;
  uint32_t error     = UINT32_MAX;
  uint32_t nominalPrescaler     = 32; //1..32
  uint32_t tempNominalPrescaler = CANFDFrequency / (tempNominalBTQ * nominalBitTime.mBitRate);

  //Find best couple
  while ((tempNominalBTQ >= 8) && (tempNominalPrescaler <= 32))
  {
    if (tempNominalPrescaler >= 1)
    {
      //calculate error of current Prescaler and BTQ
      error = (CANFDFrequency / (tempNominalBTQ * tempNominalPrescaler)) - nominalBitTime.mBitRate;
      if (error < bestError)
      {
        bestError = error;
        nominalBTQ = tempNominalBTQ;
        nominalPrescaler = tempNominalPrescaler;
      }
    }
    tempNominalBTQ--;
    tempNominalPrescaler = CANFDFrequency / (tempNominalBTQ * nominalBitTime.mBitRate);
  }


  //Step 2: Compute minimal Propagation Segment

  //minimal propagation time
  uint32_t tprop = 2 * (5 * mBusLength + mTRCV);
  nominalPropSeg = intCeil((tprop * (CANFDFrequency / 1000000)) , (nominalPrescaler * 1000));


  //Step 3: Compute minimal SJW

  //Compute NSJWmin from condition 3
  uint32_t NSJWmin = intCeil((20 * nominalBTQ * mCANFDTolerance) , 1000000);


  //Step 4: Find best bitTime configuration

  //Case 1: Propagation Segment is too long
  if ((nominalPropSeg > (nominalBTQ - 4)) || (nominalPropSeg > 48))
  {
    //PhaseSeg1,2 must be at least NJWmin
    nominalPhaseSeg1 = fmin(16 , fmax(1 , NSJWmin));
    nominalPhaseSeg2 = fmin(16 , fmax(2 , NSJWmin));

    //Add Error Code: Propagation time is too long
    bitSettingsOk += 0x10;
  }

  //Propagation Segment is within limits
  else
  {
    //Compute rest of bit (PhaseSeg1+PhaseSeg2)
    uint32_t nominalPhaseSeg = nominalBTQ - (1 + nominalPropSeg);

    //Case 2: NSJW<NSJWmin ==> NSJW smaller than min value, resynchronization may not fully synchronize the receiver
    if (nominalPhaseSeg < (2 * NSJWmin))
    {
      //Phase Seg 2 //2..16
      //choose in a way to have largest SJW
      nominalPhaseSeg2 = fmin(16 , fmax(2 , (nominalPhaseSeg / 2)));

      //Add code Error: NSJW too small
      bitSettingsOk += 0x04;
    }

    //Case 3: configuration OK
    else if (nominalPhaseSeg >= (2 * NSJWmin))
    {
      //Compute the middle of (NPS1+NPS2) ==> corresponds to biggest SJW
      uint32_t nominalMiddle = nominalPhaseSeg / 2;

      //Condition 3,4 ==> NP2max
      uint32_t cond4max = ((1000000 * nominalPhaseSeg) - (26 * nominalBTQ * mCANFDTolerance)) / (1000000 - (2 * mCANFDTolerance));
      if (cond4max < nominalMiddle) //in this case the condition is invalid
        cond4max = 16;
      NPS2max = fmin(16 , fmin((nominalPhaseSeg - 1) , fmin((nominalPhaseSeg - NSJWmin) , cond4max)));

      //Condition 3,4 ==> NP2min
      uint32_t cond4min = intCeil((26 * nominalBTQ * mCANFDTolerance) , (1000000 + (2 * mCANFDTolerance)));
      if (cond4min > nominalMiddle) //in this case the condition is invalid
        cond4min = 0;
      NPS2min = fmax(2 , fmax(NSJWmin , cond4min));


      //SamplePoint not entered ==> choose configuration with biggest SJW
      if (nominalBitTime.mSamplePoint == 0)
      {
        //Phase Seg 2 //2..16
        if (NPS2max <= nominalMiddle)
          nominalPhaseSeg2 = NPS2max;
        else if (NPS2min >= nominalMiddle)
          nominalPhaseSeg2 = NPS2min;
        else
          nominalPhaseSeg2 = nominalMiddle;
      }

      //Check if wanted SamplePoint is within bounds ==> configure wanted NBT
      else if ((nominalBitTime.mSamplePoint >= ((nominalBTQ - NPS2max) * 100 / nominalBTQ)) && (nominalBitTime.mSamplePoint <= ((nominalBTQ - NPS2min) * 100 / nominalBTQ)))
      {
        //Phase Seg 2 //2..16
        nominalPhaseSeg2 = nominalBTQ * (100 - nominalBitTime.mSamplePoint) / 100;
      }

      //wanted SamplePoint is out of bounds ==> choose closest best configuration
      else
      {
        //Phase Seg 2 //2..16
        if (nominalBitTime.mSamplePoint < ((nominalBTQ - NPS2max) * 100 / nominalBTQ))
          nominalPhaseSeg2 = NPS2max;
        else if (nominalBitTime.mSamplePoint > ((nominalBTQ - NPS2min) * 100 / nominalBTQ))
          nominalPhaseSeg2 = NPS2min;

        //Add Error Code: Wanted Nominal Sample Point out of bounds
        bitSettingsOk += 0x40;
      }
    }
    //Phase Seg 1 //1..16
    nominalPhaseSeg1 = fmin(16 , fmax(1 , (nominalPhaseSeg - nominalPhaseSeg2)));
  }

  //Recompute propagation segment
  nominalPropSeg = nominalBTQ - (1 + nominalPhaseSeg1 + nominalPhaseSeg2);

  //Compute SJW
  nominalSJW = fmin(nominalPhaseSeg1 , nominalPhaseSeg2);

  //nominal Sample Point
  nominalSamplePoint = (nominalBTQ - nominalPhaseSeg2) * 100 / nominalBTQ;


  //Step 5: Check oscillator tolerance

  uint32_t W = nominalBTQ * nominalBitTime.mBitRate * nominalPrescaler ;
  uint32_t nominalDiff = (CANFDFrequency > W) ? (CANFDFrequency - W) : (W - CANFDFrequency) ;
  nominalDiff = nominalDiff / (CANFDFrequency / 1000000); //in PPM
  if (nominalDiff > mCANFDTolerance)
    bitSettingsOk += 0x01;


  //-----------------------------------------------------------------------
  // Data configuration
  //-----------------------------------------------------------------------

  //Step 1: Find nominalPrescaler and nominalBTQ

  uint32_t dataBTQ     = 25; //5..25
  uint32_t tempDataBTQ = 25; //Start from max then decrement
  bestError = UINT32_MAX;
  error     = UINT32_MAX;
  uint32_t dataPrescaler     = 32; //1..32
  uint32_t tempDataPrescaler = CANFDFrequency / (tempDataBTQ * dataBitTime.mBitRate);

  //Find best Prescaler and BTQ
  while ((tempDataBTQ >= 5) && (tempDataPrescaler <= 32))
  {
    if (tempDataPrescaler >= 1)
    {
      //calculate error using current couple
      error = (CANFDFrequency / (tempDataBTQ * tempDataPrescaler)) - dataBitTime.mBitRate;
      if (error < bestError)
      {
        bestError = error;
        dataBTQ = tempDataBTQ;
        dataPrescaler = tempDataPrescaler;
      }
    }
    tempDataBTQ--;
    tempDataPrescaler = CANFDFrequency / (tempDataBTQ * dataBitTime.mBitRate);
  }


  //Step 2: Compute minimal SJW

  //compute minimal SJW from condition 5
  uint32_t DSJWmin = intCeil((20 * dataBTQ * mCANFDTolerance) , 1000000);


  //Step 3: Find best bitTime configuration

  //Case 1: configuration OK
  if ((2 * DSJWmin) <= (dataBTQ - 1))
  {
    //Middle of DPS1+DPS2
    uint32_t dataMiddle = (dataBTQ - 1) / 2;

    //Condition 5,7 ==> DPS2max
    uint32_t tempA = 1000000 * dataBTQ;
    uint32_t tempB = (1000000 * (1 + fmax(0 , ((nominalPrescaler / dataPrescaler) - 1)))) + (8 * mCANFDTolerance * dataBTQ) + (2 * mCANFDTolerance * (2 * nominalBTQ - nominalPhaseSeg2) * (nominalPrescaler / dataPrescaler));
    uint32_t cond7max = (tempA > tempB) ? ((tempA - tempB) / (1000000 + 2 * mCANFDTolerance)) : 8;
    if (cond7max < dataMiddle)
      cond7max = 8;
    DPS2max = fmin(8 , fmin(((dataBTQ - 1) - 1) , fmin(((dataBTQ - 1) - DSJWmin) , cond7max)));

    //Condition5,7 ==> DPS2min
    tempA = (8 * mCANFDTolerance * dataBTQ) + (2 * mCANFDTolerance * (2 * nominalBTQ - nominalPhaseSeg2) * intCeil(nominalPrescaler , dataPrescaler));
    tempB = 1000000 * fmax(0 , (intCeil(nominalPrescaler, dataPrescaler) - 1));
    uint32_t cond7min = (tempA > tempB) ? (intCeil((tempA - tempB) , (1000000 - 2 * mCANFDTolerance))) : 0;
    if (cond7min > dataMiddle)
      cond7min = 0;
    DPS2min = fmax(2 , fmax(DSJWmin , cond7min));

    //Sample Point not entered ==> best Configuration
    if (dataBitTime.mSamplePoint == 0)
    {
      //Phase Seg 2 //2..16
      //choose in a way to have largest SJW
      if (DPS2max <= dataMiddle)
        dataPhaseSeg2 = DPS2max;
      else if (DPS2min >= dataMiddle)
        dataPhaseSeg2 = DPS2min;
      else
        dataPhaseSeg2 = dataMiddle;
    }

    //Check if wanted NSamplePoint is within limits ==> configure wanted NBT
    else if ((dataBitTime.mSamplePoint >= ((dataBTQ - DPS2max) * 100 / dataBTQ)) && (dataBitTime.mSamplePoint <= ((dataBTQ - DPS2min) * 100 / dataBTQ)))
    {
      //Phase Seg 2 //2..16
      dataPhaseSeg2 = dataBTQ * (100 - dataBitTime.mSamplePoint) / 100;
    }

    //NSamplePoint isn't within limits ==> choose closest best configuration
    else
    {
      //Phase Seg 2 //2..16
      if (dataBitTime.mSamplePoint < DPS2min)
        dataPhaseSeg2 = DPS2min;
      else if (dataBitTime.mSamplePoint > DPS2max)
        dataPhaseSeg2 = DPS2max;

      //Add Error Code: Wanted Data Sample Point out of bounds
      bitSettingsOk += 0x80;
    }
  }

  //Case 2: DSJW<DSJWmin ==> DSJW smaller than min value, resynchronization may not fully synchronize the receiver
  else if ((dataBTQ - 1) < (2 * DSJWmin))
  {
    //Phase Seg 2 //2..8
    //choose in a way to have largest SJW
    dataPhaseSeg2 = fmin(8 , fmax(2 , ((dataBTQ - 1) / 2)));

    //Add code Error: DSJW too small
    bitSettingsOk += 0x08;
  }

  //Phase Seg 1 //1..8
  dataPhaseSeg1 = fmin(8 , fmax(1 , (dataBTQ - (1 + dataPhaseSeg2))));

  //Add what's left to prop seg
  dataPropSeg = dataBTQ - (1 + dataPhaseSeg1 + dataPhaseSeg2);

  //data Sample Point
  dataBitTime.mSamplePoint = ((dataBTQ - dataPhaseSeg2) * 100 / dataBTQ);

  //Compute SJW
  dataSJW = fmin(dataPhaseSeg1, dataPhaseSeg2);


  //Step 4: Check oscillator tolerance

  W = dataBTQ * dataBitTime.mBitRate * dataPrescaler ;
  uint32_t dataDiff = (CANFDFrequency > W) ? (CANFDFrequency - W) : (W - CANFDFrequency) ;
  dataDiff = dataDiff / (CANFDFrequency / 1000000); //in PPM
  if (dataDiff > mCANFDTolerance)
    bitSettingsOk += 0x02;


  //-----------------------------------------------------------------------
  // Write configuration to C1NBTCFG_REGISTER and C1DBTCFG_REGISTER
  //-----------------------------------------------------------------------

  uint32_t data = (nominalPrescaler - 1) & 0x000000FF;
  data = (data << 8) | ((nominalPhaseSeg1 + nominalPropSeg - 1) & 0x000000FF);
  data = (data << 8) | ((nominalPhaseSeg2 - 1) & 0x000000FF);
  data = (data << 8) | ((nominalSJW - 1) & 0x000000FF);
  writeRegister(C1NBTCFG_REGISTER, data);

  data = (dataPrescaler - 1) & 0x000000FF;
  data = (data << 8) | ((dataPhaseSeg1 + dataPropSeg - 1) & 0x000000FF);
  data = (data << 8) | ((dataPhaseSeg2 - 1) & 0x000000FF);
  data = (data << 8) | ((dataSJW - 1) & 0x000000FF);
  writeRegister(C1DBTCFG_REGISTER, data);


  //-----------------------------------------------------------------------
  // Save values
  //-----------------------------------------------------------------------

  nominalBitTime.mBTQ         = (uint8_t) nominalBTQ;
  nominalBitTime.mPrescaler   = (uint8_t) nominalPrescaler;
  nominalBitTime.mPropSeg     = (uint8_t) nominalPropSeg;
  nominalBitTime.mPhaseSeg1   = (uint8_t) nominalPhaseSeg1;
  nominalBitTime.mPhaseSeg2   = (uint8_t) nominalPhaseSeg2;
  nominalBitTime.mSJW         = (uint8_t) nominalSJW;
  nominalBitTime.mSamplePoint = nominalSamplePoint;

  dataBitTime.mBTQ         = (uint8_t) dataBTQ;
  dataBitTime.mPrescaler   = (uint8_t) dataPrescaler;
  dataBitTime.mPropSeg     = (uint8_t) dataPropSeg;
  dataBitTime.mPhaseSeg1   = (uint8_t) dataPhaseSeg1;
  dataBitTime.mPhaseSeg2   = (uint8_t) dataPhaseSeg2;
  dataBitTime.mSJW         = (uint8_t) dataSJW;
  dataBitTime.mSamplePoint = dataSamplePoint;


  return bitSettingsOk;
}



//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//--- Configure TEF
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::configureTEF (const bool inState, const uint32_t inFIFOSize, const bool inTimeStamp)
{
  uint32_t data = readRegister(C1CON_REGISTER);

  if ((data & 0x07000000) ^ 0x04000000) //if true then we're not in configuration mode
    return 0;

  else //we're in configuration mode
  {
    //set STEF to 0
    data &= 0xFFF7FFFF;

    if (inState == 1)
    {
      //set STEF to 1
      data |= 0x00080000;
      writeRegister(C1CON_REGISTER, data);

      //Set FIFO size
      uint32_t fifoSize = inFIFOSize - 1;
      if (fifoSize > 31) fifoSize = 31;
      data = readRegister(C1TEFCON_REGISTER);
      data &= 0xE0FFFFFF; //clear FSize
      data |= (fifoSize << 24); //Set Fsize to fifoSize

      //Enable TimeStamp
      if(inTimeStamp)
        data |= 0x00000020;
        
      writeRegister(C1TEFCON_REGISTER, data);
    }

    else if (inState == 0)
      writeRegister(C1CON_REGISTER, data);

    return 1;
  }
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::isTEFFIFONotFull (void)
{
  uint32_t data = readRegister(C1TEFSTA_REGISTER);
  if (data & 0x00000004) //TEFFIF=1 ==> full
    return 0;
  else //TEFFIF=0 ==> not full
    return 1;
}

bool MCP2517FD::resetTEF (void)
{
  uint32_t data = readRegister(C1TEFCON_REGISTER);

  // FRESET=1 ==> do nothing
  if(data & 0x00000400)
    return 1;
  
  else
  {
    //set FRESET to reset TEF
    data |= 0x00000400;
    
    writeRegister(C1TEFCON_REGISTER, data);

    //wait for FRESET to clear
    uint8_t k = 0;
    while (readRegister(C1TEFCON_REGISTER) & 0x00000400)
    {
      k++;
      if (k == 100)
        return 0;
    }

    return 1;
  }
}



//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//--- Configure TxQ
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::configureTXQ (const bool inState, const uint32_t inFIFOSize, const uint32_t inPayloadSize,
                              const uint32_t inPriority)
{
  uint32_t data = readRegister(C1CON_REGISTER);

  if (((data & 0x07000000) ^ 0x06000000) == 0) //if true then we're not in configuration mode
    return 0;
    
  else //we're in configureation mode
  {
    //set TXQEN to 0
    data &= 0xFFEFFFFF;

    if (inState == 1)
    {
      //set TXQEN to 1
      data |= 0x00100000;
      writeRegister(C1CON_REGISTER, data);

      //Set FIFO size
      uint32_t fifoSize = inFIFOSize - 1;
      if (fifoSize > 31) fifoSize = 31;
      data = readRegister(C1TXQCON_REGISTER);
      data &= 0xE0FFFFFF; //clear FSize
      data |= (fifoSize << 24); //Set Fsize to fifoSize

      //Set Payload size
      uint32_t payloadSize;
      if (inPayloadSize <= 8)
        payloadSize = 0;
      else
        switch (inPayloadSize)
        {
          case 12:
            payloadSize = 0x001;
            break;
          case 16:
            payloadSize = 0x010;
            break;
          case 20:
            payloadSize = 0x011;
            break;
          case 24:
            payloadSize = 0x100;
            break;
          case 32:
            payloadSize = 0x101;
            break;
          case 48:
            payloadSize = 0x110;
            break;
          case 64:
            payloadSize = 0x111;
            break;
          default:
            break;
        }
      data &= 0x1FFFFFFF; //clear PLSize
      data |= (payloadSize << 29); //Set PLsize to payloadSize

      //Set priority 0(low)..100(high)
      data &= 0xFFE0FFFF; //clear TXPRI
      uint32_t priority = inPriority;
      if(inPriority > 100) priority=100;
      priority = (priority * 32) / 100;
      data |= ((priority & 0x0000001F) << 16);
     
      writeRegister(C1TXQCON_REGISTER, data);

    }
    else if (inState == 0)
      writeRegister(C1CON_REGISTER, data);

    return 1;
  }
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::isTXQFIFONotFull (void)
{
  uint32_t data = readRegister(C1TXQSTA_REGISTER);
  if (data & 0x00000001) //TXQNIF=1 ==> Not full
    return 1;
  else //TXQNIF=0 ==> full
    return 0;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void MCP2517FD::changeTXQRetransmissionAttempts (const uint32_t inRetransmissionAttempts)
{
  //activate retransmission attempts from C1CON
  uint32_t data = readRegister(C1CON_REGISTER);
  data &= 0xFFFEFFFF; //clear RTXAT
  data |= 0x00010000; //set RTXAT => activate retransmission attempts
  writeRegister(C1CON_REGISTER, data);
  
  //set retransmission attempts for TXQ
  data = readRegister(C1TXQCON_REGISTER);
  data &= 0xFF9FFFFF; //clear TXAT => disable retransmission attemps
  switch(inRetransmissionAttempts)
  {
  case 0: //disable retransmission
    //do nothing already disabled TXAT=00
    break;
  case 1: //3 retransmission attempts
    data |= 0x00200000; //TXAT=01
    break;
  default: //unlimited retransmission attempts
    data |= 0x00600000; //TXAT=11
    break;
  }
  writeRegister(C1TXQCON_REGISTER, data);
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::resetTXQ (void)
{
  uint32_t data = readRegister(C1TXQCON_REGISTER);

  // FRESET=1 ==> do nothing
  if(data & 0x00000400)
    return 1;
  
  else
  {
    //set FRESET to reset TEF
    data |= 0x00000400;
    
    writeRegister(C1TXQCON_REGISTER, data);

    //wait for FRESET to clear
    uint8_t k = 0;
    while (readRegister(C1TXQCON_REGISTER) & 0x00000400)
    {
      k++;
      if (k == 100)
        return 0;
    }

    return 1;
  }
}



//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//--- Configure Transmit and receive FIFOs
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::configureFIFOm (const uint16_t m, const bool inState, const uint32_t inFIFOSize, const uint32_t inPayloadSize,
                                const uint32_t inPriority, const bool inTimeStamp)
{
  if (m < 1 || m > 31)
    return 0; //FIFOm doesn't exist

  else
  {
    uint32_t data = readRegister(C1CON_REGISTER);

    if (((data & 0x07000000) ^ 0x06000000) == 0) //we're not in configuration mode
      return 0;

    else //we're in configuration mode
    {
      uint16_t adress = C1FIFOCON1_REGISTER + 12 * (m - 1);

      data = readRegister(adress);
      //configure to TX or Rx
      data &= 0xFFFFFF7F; //configure to Rx
      if (inState == 1)
        data |= 0x00000080; //configure to Tx

      //Set FIFO size
      uint32_t fifoSize = inFIFOSize - 1;
      if (fifoSize > 31) fifoSize = 31;
      data &= 0xE0FFFFFF; //clear FSize
      data |= (fifoSize << 24); //Set Fsize to fifoSize

      //Set Payload size
      uint32_t payloadSize;
      if (inPayloadSize <= 8)
        payloadSize = 0;
      else
        switch (inPayloadSize)
        {
          case 12:
            payloadSize = 0x001;
            break;
          case 16:
            payloadSize = 0x010;
            break;
          case 20:
            payloadSize = 0x011;
            break;
          case 24:
            payloadSize = 0x100;
            break;
          case 32:
            payloadSize = 0x101;
            break;
          case 48:
            payloadSize = 0x110;
            break;
          case 64:
            payloadSize = 0x111;
            break;
          default:
            break;
        }
      data &= 0x1FFFFFFF; //clear PLSize
      data |= (payloadSize << 29); //Set PLsize to payloadSize

      //Set priority: 0(low)..100(high)
      data &= 0xFFE0FFFF; //clear TXPRI
      uint32_t priority = inPriority;
      if(inPriority > 100) priority=100;
      priority = (priority * 32) / 100;
      data |= ((priority & 0x0000001F) << 16);

      //receive timestamp
      if(inTimeStamp && !inState)
      {
        data &= 0xFFFFFFDF; //clear RXTSEN
        data |= 0x00000020; //set RXTSEN
      }

      writeRegister(adress, data);

      return 1;
    }
  }
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::isTransmitFIFOmNotFull (const uint16_t m)
{
  if(m < 1 || m > 32)
    return 0; //FIFOm doesn't exist
    
  uint32_t data = readRegister(C1FIFOSTA1_REGISTER + 12 * (m - 1));

  //Transmit FIFO
  if (isFIFOmTXorRX(m) == 1)
  {
    if (data & 0x00000001) //TXQNIF=1 ==> Not full
      return 1;
    else //TXQNIF=0 ==> full
      return 0;
  }

  return 0;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::isReceiveFIFOmNotEmpty (const uint16_t m)
{
  if(m < 1 || m > 32)
    return 0; //FIFOm doesn't exist
    
  uint32_t data = readRegister(C1FIFOSTA1_REGISTER + 12 * (m - 1));

  //Receive FIFO
  if (isFIFOmTXorRX(m) == 0)
  {
    if (data & 0x00000001) //TXQNIF=1 ==> not empty
      return 1;
    else //TXQNIF=0 ==> empty
      return 0;
  }

  return 0;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::isFIFOmTXorRX (const uint16_t m)
{
  if(m < 1 || m > 32)
    return 0; //FIFOm doesn't exist
    
  uint32_t data = readRegister(C1FIFOCON1_REGISTER + 12 * (m - 1));
  if (data & 0x00000080) //TXEN=1 ==> TX
    return 1;
  else //TXEN=0 ==> RX
    return 0;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::changeFIFOmRetransmissionAttempts (const uint16_t m, const uint32_t inRetransmissionAttempts)
{
  if(m < 1 || m > 32)
    return 0; //FIFOm doesn't exist
    
  //activate retransmission attempts from C1CON
  uint32_t data = readRegister(C1CON_REGISTER);
  data &= 0xFFFEFFFF; //clear RTXAT
  data |= 0x00010000; //set RTXAT => activate retransmission attempts
  writeRegister(C1CON_REGISTER, data);

  //set retransmission attempts for TXQ
  uint16_t adress = C1FIFOCON1_REGISTER + 12 * (m - 1);
  data = readRegister(adress);
  data &= 0xFF9FFFFF; //clear TXAT => disable retransmission attemps
  switch(inRetransmissionAttempts)
  {
  case 0: //disable retransmission
    //do nothing already disabled TXAT=00
    break;
  case 1: //3 retransmission attempts
    data |= 0x00200000; //TXAT=01
    break;
  default: //unlimited retransmission attempts
    data |= 0x00600000; //TXAT=11
    break;
  }
  writeRegister(adress, data);
  
  return 1;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::resetFIFOm (const uint16_t m)
{
  if(m < 1 || m > 32)
    return 0;

  uint16_t adress = C1FIFOCON1_REGISTER + 12 * (m - 1);
  uint32_t data = readRegister(adress);

  // FRESET=1 ==> do nothing
  if(data & 0x00000400)
    return 1;
  
  else
  {
    //set FRESET to reset TEF
    data |= 0x00000400;
    
    writeRegister(adress, data);

    //wait for FRESET to clear
    uint8_t k = 0;
    while (readRegister(adress) & 0x00000400)
    {
      k++;
      if (k == 100)
        return 0;
    }

    return 1;
  }
}



//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// Configure filters
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::configureFilterm (const uint16_t m, const uint32_t inFIFO, const bool inMatchExtAndStandId,
                                  const uint32_t inStandId, const uint32_t inStandIdRange, const uint32_t inExtId,
                                  const uint32_t inExtIdRange, const bool inMatchExtOrStandId)
{
  if (m < 1 || m > 32)
    return 0;

  else
  {
    uint16_t conReg  = C1FLTCON0_REGISTER  + 4 * ((m - 1) / 4);
    uint16_t objReg  = C1FLTOBJ0_REGISTER  + 8 * (m - 1);
    uint16_t maskReg = C1MASK0_REGISTER    + 8 * (m - 1);

    uint32_t data = readRegister (conReg);

    //disable filter
    switch ((m - 1) % 4)
    {
      case 0:
        data &= 0xFFFFFF7F;
        break;
      case 1:
        data &= 0xFFFF7FFF;
        break;
      case 2:
        data &= 0xFF7FFFFF;
        break;
      case 3:
        data &= 0x7FFFFFFF;
        break;
    }
    writeRegister(conReg, data);

    // save messages that pass the filter to FIFOx
    switch ((m - 1) % 4)
    {
      case 0:
        data &= 0xFFFFFFE0;
        data |= (inFIFO & 0x0000001F);
        break;
      case 1:
        data &= 0xFFFFE0FF;
        data |= ((inFIFO & 0x0000001F) << 8);
        break;
      case 2:
        data &= 0xFFE0FFFF;
        data |= ((inFIFO & 0x0000001F) << 16);
        break;
      case 3:
        data &= 0xE0FFFFFF;
        data |= ((inFIFO & 0x0000001F) << 24);
        break;
    }
    writeRegister(conReg, data);

    data = readRegister(objReg);
    //if MIDE=1 then match only extended or standard id
    data &= 0xBFFFFFFF; //match only standard id
    if (inMatchExtOrStandId == 1)
      data |= 0x40000000; //match only extended id

    //standard id filter
    data &= 0xFFFFF000; //clear standard id
    data |= (inStandId & 0x00000FFF); //set standard id

    //extended id filter
    data &= 0xE0000FFF; //clear extended id
    data |= ((inExtId & 0x0001FFFF) << 12);

    writeRegister(objReg, data);

    data = readRegister(maskReg);
    //match both standard and extended ids or just one of them
    data &= 0xBFFFFFFF; //match both
    if (inMatchExtAndStandId == 0)
      data |= 0x40000000; //match one of them

    //standard id max range
    data &= 0xFFFFF000; //clear standard id
    data |= (inStandIdRange & 0x00000FFF); //set standard id max range

    //extended id filter
    data &= 0xE0000FFF; //clear extended id
    data |= ((inExtIdRange & 0x0001FFFF) << 12); //set extended id max range

    writeRegister(maskReg, data);

    //enable filter
    data = readRegister (conReg);
    switch ((m - 1) % 4)
    {
      case 0:
        data |= 0x00000080;
        break;
      case 1:
        data |= 0x00008000;
        break;
      case 2:
        data |= 0x00800000;
        break;
      case 3:
        data |= 0x80000000;
        break;
    }
    writeRegister(conReg, data);

    return 1;
  }
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::enableFilterm (const uint16_t m)
{
  if (m < 1 || m > 32)
    return 0;

  else
  {
    uint16_t conReg  = C1FLTCON0_REGISTER  + 4 * ((m - 1) / 4);

    uint32_t data = readRegister (conReg);

    //enable filter
    data = readRegister (conReg);
    switch ((m - 1) % 4)
    {
      case 0:
        if(data & 0x00000080) //filter enabled
          return 1;
        else
          data |= 0x00000080;
        break;
      case 1:
        if(data & 0x00008000) //filter enabled
          return 1;
        else
          data |= 0x00008000;
        break;
      case 2:
        if(data & 0x00800000) //filter enabled
          return 1;
        else
          data |= 0x00800000;
        break;
      case 3:
        if(data & 0x80000000) //filter enabled
          return 1;
        else
          data |= 0x80000000;
        break;
    }
    writeRegister(conReg, data);

    return 1;
  }
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::disableFilterm (const uint16_t m)
{
  if (m < 1 || m > 32)
    return 0;

  else
  {
    uint16_t conReg  = C1FLTCON0_REGISTER  + 4 * ((m - 1) / 4);

    uint32_t data = readRegister (conReg);
    
    //disable filter
    switch ((m - 1) % 4)
    {
      case 0:
        data &= 0xFFFFFF7F;
        break;
      case 1:
        data &= 0xFFFF7FFF;
        break;
      case 2:
        data &= 0xFF7FFFFF;
        break;
      case 3:
        data &= 0x7FFFFFFF;
        break;
    }

    return 1;
  }
}



//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// Configure operation mode
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void MCP2517FD::configureMode (const ConfMode inMode)
{
  uint32_t data = readRegister (C1CON_REGISTER);
  data &= 0xF8FFFFDF; //normal mode

  switch (inMode)
  {
    case SleepMode:
      data |= 0x01000000;
      break;
    case InternalLoopbackMode:
      data |= 0x02000000;
      break;
    case ListenOnlyMode:
      data |= 0x03000000;
      break;
    case ConfigurationMode:
      data |= 0x04000000;
      break;
    case ExternalLoopbackMode:
      data |= 0x05000000;
      break;
    case CANMode:
      data |= 0x06000000;
      break;
    default:
      break;
  }

  writeRegister(C1CON_REGISTER, data);
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

MCP2517FD::ConfMode MCP2517FD::getConfigurationMode (void)
{
  uint32_t data = readRegister (C1CON_REGISTER);
  data &= 0x00E00000;

  switch (data)
  {
    case 0x00000000:
      return NormalMode;
      break;
    case 0x00200000:
      return SleepMode;
      break;
    case 0x00400000:
      return InternalLoopbackMode;
      break;
    case 0x00600000:
      return ListenOnlyMode;
      break;
    case 0x00800000:
      return ConfigurationMode;
      break;
    case 0x00A00000:
      return ExternalLoopbackMode;
      break;
    case 0x00C00000:
      return CANMode;
      break;
  }
}



//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// Send and receive frames
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::sendFrame (const uint32_t inChannel, const CANFDFrame &inFrame, const bool inDontSend)
{
  uint16_t confAdress;
  uint16_t sendAdress;

  if (inChannel == 0) //TxQ
  {
    //check if TxQ is disabled
    if ((readRegister(C1CON_REGISTER) & 0x00100000) == 0)
      return 0;

    confAdress = C1TXQCON_REGISTER;
    sendAdress = 0x400 + (uint16_t)readRegister(C1TXQUA_REGISTER);

    //check if FIFO is full
    if (isTXQFIFONotFull() == 0)
      return 0;
  }

  else //FIFO_{inChannel}
  {
    if (inChannel > 32)
      return 0;

    confAdress = C1FIFOCON1_REGISTER + 12 * (inChannel - 1);
    sendAdress = 0x400 + (uint16_t)readRegister(C1FIFOUA1_REGISTER + 12 * (inChannel - 1));

    //check that FIFOm is TX
    if (isFIFOmTXorRX(inChannel) == 0)
      return 0;

    //verify FIFO not full
    if (isTransmitFIFOmNotFull(inChannel) == 0)
      return 0;
  }

  //wait for FRESET to clear if it's set
  uint8_t k = 0;
  while (readRegister(confAdress) & 0x00000400)
  {
    k++;
    if (k == 100)
      return 0;
  }

  //put message in the FIFO of TxQ or FIFOm
  uint32_t data = inFrame.getID();
  writeRegister(sendAdress, data);
  data = (0x0000000F & (uint32_t)inFrame.getDLC());
  if(inFrame.getIDE()) data |= 0x00000010;
  if(inFrame.getRTR()) data |= 0x00000020;
  if(inFrame.getBRS()) data |= 0x00000040;
  if(inFrame.getFDF()) data |= 0x00000080;
  if(inFrame.getESI()) data |= 0x00000100;
  writeRegister(sendAdress + 4, data);
  for(unsigned int i = 0; i <= (dataLengthFromDLC(inFrame.getDLC()) / 4); i++)
  {
    data = 0;
    for(unsigned int j = 4; j >= 1; j--)
    {
      if((4*i + j-1) < dataLengthFromDLC(inFrame.getDLC()))
        data |= (((uint32_t)inFrame.getData()[(4*i + j-1)] & 0x000000FF) << (8*(j-1)));
    }
    writeRegister(sendAdress + 8 + 4*i, data);
  }
  
  //increment FIFO
  data = readRegister(confAdress);
  data |= 0x00000100;
  writeRegister(confAdress, data);

  //request transmission
  if(inDontSend == 0)
  {
    //send message
    data = readRegister(confAdress);
    data |= 0x00000200;
    writeRegister(confAdress, data);
  }

  return 1;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::requestTransmission (const uint8_t inTransmitMode, const uint32_t inChannel)
{
  uint32_t data = readRegister(C1TXREQ_REGISTER);
  
  if(inTransmitMode == 0) //request transmission of all messages
    data = 0xFFFFFFFF;
    
  else if(inTransmitMode == 1) //request transmission of a specific channel (inChannel in INT format)
  {
    if(inChannel >= 32)
      return 0;
    data |= (1 << inChannel);
  }
    
  else //request transmission of multiple channels (inChannel in HEX format)
    data |= inChannel;

  writeRegister(C1TXREQ_REGISTER, data);
  return 1;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

CANFDFrame* MCP2517FD::receiveFrame (const uint32_t inChannel)
{
  if(inChannel >= 1 && inChannel <= 32 && isFIFOmTXorRX(inChannel) == 0 && isReceiveFIFOmNotEmpty(inChannel) == 1)
  {
    uint32_t ID;
    bool IDE, FDF, RTR, BRS, ESI;
    uint8_t DLC;
    embeddedvector <uint8_t> Data;

    uint16_t confAdress = C1FIFOCON1_REGISTER + 12 * (inChannel - 1);
    uint16_t receiveAdress = 0x400 + (uint16_t)readRegister(C1FIFOUA1_REGISTER + 12 * (inChannel - 1));
          
    //read message from the buffer FIFO
    uint32_t data = readRegister(receiveAdress);
    ID = data & 0x1FFFFFFF;
    
    data = readRegister(receiveAdress + 4);
    (data & 0x00000100)? ESI=1 : ESI=0;
    (data & 0x00000080)? FDF=1 : FDF=0;
    (data & 0x00000040)? BRS=1 : BRS=0;
    (data & 0x00000020)? RTR=1 : RTR=0;
    (data & 0x00000010)? IDE=1 : IDE=0;
    DLC = data & 0x0000000F;
    
    for(unsigned int i = 0; i <= (dataLengthFromDLC(DLC) / 4); i++)
    {
      if(readRegister(confAdress) & 0x00000020) //RXTSEN=1 ==> timestamp
        data = readRegister(receiveAdress + 12 + 4*i);
      else
        data = readRegister(receiveAdress + 8 + 4*i);
      for(unsigned int j = 0; j <= 3; j++)
      {
        if((4*i + j) < dataLengthFromDLC(DLC))
          Data.push((uint8_t)(0x000000FF & (data >> (8*j))));
      }
    }

    //construct frame
    CANFDFrame* frame = new CANFDFrame(FDF, IDE, RTR, ID, BRS, ESI, dataLengthFromDLC(DLC), Data);
    
    //increment FIFO
    data = readRegister(confAdress);
    data |= 0x00000100;
    writeRegister(confAdress, data);

    return frame;
  }
  else
    return NULL;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::abortTransmission (const uint32_t inChannel)
{
  uint16_t confAdress;

  if (inChannel == 0) //TxQ
  {
    //check if TxQ is disabled
    if ((readRegister(C1CON_REGISTER) & 0x00100000) == 0)
      return 0;

    confAdress = C1TXQCON_REGISTER;
  }

  else //FIFO_{inChannel}
  {
    if (inChannel > 32)
      return 0;

    confAdress = C1FIFOCON1_REGISTER + 12 * (inChannel - 1);

    //check that FIFOm is TX
    if (isFIFOmTXorRX(inChannel) == 0)
      return 0;
  }

  //clear TXREQ bit
  uint32_t data = readRegister(confAdress);
  data &= 0xFFFFFDFF;
  writeRegister(confAdress, data);
}



//————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// Time Stamping
//————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
  
bool MCP2517FD::enableTimeStamping (void)
{
  uint32_t data = readRegister(C1TSCON_REGISTER);

  if(data & 0x00010000)
    return 1;

  // set TBCEN
  data |= 0x00010000;

  writeRegister(C1TSCON_REGISTER, data);

  return 1;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::disableTimeStamping (void)
{
  uint32_t data = readRegister(C1TSCON_REGISTER);

  if((data & 0x00010000))
    return 1;

  // clear TBCEN
  data &= 0xFFFEFFFF;

  writeRegister(C1TSCON_REGISTER, data);

  return 1;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool MCP2517FD::configureTimeStampingPrescaler (uint32_t inPrescaler)
{
  if(inPrescaler > 1024)
    return 0;

  uint32_t data = readRegister(C1TSCON_REGISTER);

  //clear TBC Prescaler
  data &= 0xFFFFFC00;
  for(uint8_t i=0;i<10;i++)
    data |= ((inPrescaler >> i) & 0x00000001);

  writeRegister(C1TSCON_REGISTER, data);
}

uint32_t MCP2517FD::getTBC (void)
{
  return readRegister(C1TBC_REGISTER);
}



//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//  Private methods
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void MCP2517FD::writeRegister (const uint16_t inAddress, const uint32_t inValue)
{
  const uint16_t command = (inAddress & 0x0FFF) | (0b0010 << 12) ;
  digitalWrite (mCS, LOW) ;
  mSPI.beginTransaction (mSPISettings) ;
  mSPI.transfer16 (command) ; // Command
  mSPI.transfer ((byte) (inValue & 0xFF)) ; // Data
  mSPI.transfer ((byte) (inValue >>  8)) ; // Data
  mSPI.transfer ((byte) (inValue >> 16)) ; // Data
  mSPI.transfer ((byte) (inValue >> 24)) ; // Data
  mSPI.endTransaction () ;
  digitalWrite (mCS, HIGH) ;
}



//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// Public methods
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint32_t MCP2517FD::readRegister (const uint16_t inAddress)
{
  const uint16_t command = (inAddress & 0x0FFF) | (0b0011 << 12) ;
  digitalWrite (mCS, LOW) ;
  mSPI.beginTransaction (mSPISettings) ;
  mSPI.transfer16 (command) ; // Command
  uint32_t result = mSPI.transfer (0x00) ;
  result |= ((uint32_t) mSPI.transfer (0x00)) <<  8 ;
  result |= ((uint32_t) mSPI.transfer (0x00)) << 16 ;
  result |= ((uint32_t) mSPI.transfer (0x00)) << 24 ;
  mSPI.endTransaction () ;
  digitalWrite (mCS, HIGH) ;
  return result ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint32_t MCP2517FD::intCeil(const uint32_t a, const uint32_t b)
{
  if (a % b)
    return (a / b) + 1;
  else
    return a / b;
}

