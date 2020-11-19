#include "odie.h"


#define GPS_SERVER_no

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

TSpi Spi( hspi1 );
TGps Gps( huart1 );
TOdie Odie;
TSystem System;

void TOdie::Setup()
{
//  NvDataInit();
//  CalDataInit();
  Gps.Init();

  SetPin( LED1_GREEN_GPIO_Port, LED1_GREEN_Pin );
  SetPin( LED2_GREEN_GPIO_Port, LED2_GREEN_Pin );

  if( Radio433.Init() )
  {
    Radio433.Receive();
  }
  else
  {
    Hmi1Error();
  }

  if( Radio868.Init() )
  {
    Radio868.Receive();
  }
  else
  {
    Hmi2Error();
  }
}

void TOdie::Loop()
{
  HmiLoop();
  Gps.Loop();

  if( Radio433Flag )
  {
    Radio433Flag = false;
    Radio433.Interrupt();
  }
  if( Radio868Flag )
  {
    Radio868Flag = false;
    Radio868.Interrupt();
  }

#ifdef GPS_SERVER
  auto const Tick = HAL_GetTick();
  static uint32_t LastTick;
  if(( Tick > LastTick ) && ( Tick % 1000 ) == 0 )
  {
    LastTick = Tick;
    auto const DataRMC = Gps.GetRMC();
    Radio433.Transmit( &DataRMC, sizeof( DataRMC ));
    Radio868.Transmit( &DataRMC, sizeof( DataRMC ));

    UsbPrintf( "%4d %3d.%u %06u %3u.%03u : %02u %u.%05u %c %03u %u.%05u %c\n",
      0, 0, 0,
      DataRMC.Time,
      DataRMC.Speed / 1000, DataRMC.Speed % 1000,
      DataRMC.LatDeg, DataRMC.LatMin, DataRMC.LatSec, DataRMC.LatSouth ? 'S' : 'N',
      DataRMC.LonDeg, DataRMC.LonMin, DataRMC.LonSec, DataRMC.LonWest ? 'W' : 'E' );
  }
#endif
}

void TOdie::Radio433Event( TRadioEvent const Event )
{
  union
  {
    uint8_t Buffer[ 256 ];
    TGps::TDataRMC DataRMC;
    int16_t Temp[8];
  };

  if( Event == TRadioEvent::RxDone )
  {
    auto const Snr = Radio433.GetSnr();
    auto const Rssi = Radio433.GetRssi();
    auto const Length = Radio433.ReadPacket( Buffer, sizeof( Buffer ));

    if( Length == sizeof( TGps::TDataRMC ))
    {
      Hmi1Blue( 100 );
      UsbPrintf( "433 %4d %3d.%u %06u %3u.%03u : %02u %u.%05u %c %03u %u.%05u %c\n",
        Rssi, Snr / 10, abs(Snr) % 10,
        DataRMC.Time,
        DataRMC.Speed / 1000, DataRMC.Speed % 1000,
        DataRMC.LatDeg, DataRMC.LatMin, DataRMC.LatSec, DataRMC.LatSouth ? 'S' : 'N',
        DataRMC.LonDeg, DataRMC.LonMin, DataRMC.LonSec, DataRMC.LonWest ? 'W' : 'E' );
    }
    else if( Length == sizeof( Temp ))
    {
      UsbPrintf( "433 Rssi:%4d Snr:%3d.%u Len:%u\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
      for( auto Index = 0; Index < 8; Index++ )
      {
        UsbPrintf( "%d, %d.%u\n", Index, Temp[ Index ] / 10, Temp[ Index ] % 10 );
      }
    }
    else
    {
      UsbPrintf( "433 Rssi:%4d Snr:%3d.%u Len:%u Length error\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
    }
  }

  if( Event == TRadioEvent::TxDone )
  {
    Hmi1Blue( 100 );
    Radio433.Receive();
  }

  if( Event == TRadioEvent::Timeout )
  {
    Hmi1Error();
  }

  if( Event == TRadioEvent::CrcError )
  {
    auto const Snr = Radio433.GetSnr();
    auto const Rssi = Radio433.GetRssi();
    auto const Length = Radio433.ReadPacket( Buffer, sizeof( Buffer ));

    UsbPrintf( "433 Rssi:%4d Snr:%3d.%u Len:%u CRC Error\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
  }

  if( Event == TRadioEvent::NoCrc )
  {
    auto const Snr = Radio433.GetSnr();
    auto const Rssi = Radio433.GetRssi();
    auto const Length = Radio433.ReadPacket( Buffer, sizeof( Buffer ));

    UsbPrintf( "433 Rssi:%4d Snr:%3d.%u Len:%u No CRC\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
  }
}

void TOdie::Radio868Event( TRadioEvent const Event )
{
  union
  {
    uint8_t Buffer[ 256 ];
    TGps::TDataRMC DataRMC;
  };

  if( Event == TRadioEvent::RxDone )
  {
    auto const Snr = Radio868.GetSnr();
    auto const Rssi = Radio868.GetRssi();
    auto const Length = Radio868.ReadPacket( Buffer, sizeof( Buffer ));

    if( Length == sizeof( TGps::TDataRMC ))
    {
      Hmi2Blue( 100 );
      UsbPrintf( "868 %4d %3d.%u %06u %3u.%03u : %02u %u.%05u %c %03u %u.%05u %c\n",
        Rssi, Snr / 10, abs(Snr) % 10,
        DataRMC.Time,
        DataRMC.Speed / 1000, DataRMC.Speed % 1000,
        DataRMC.LatDeg, DataRMC.LatMin, DataRMC.LatSec, DataRMC.LatSouth ? 'S' : 'N',
        DataRMC.LonDeg, DataRMC.LonMin, DataRMC.LonSec, DataRMC.LonWest ? 'W' : 'E' );
    }
    else
    {
      UsbPrintf( "868 Rssi:%4d Snr:%3d.%u Len:%u Length error\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
    }
  }

  if( Event == TRadioEvent::TxDone )
  {
    Hmi2Blue( 100 );
    Radio868.Receive();
  }

  if( Event == TRadioEvent::Timeout )
  {
    Hmi2Error();
  }

  if( Event == TRadioEvent::CrcError )
  {
    auto const Snr = Radio868.GetSnr();
    auto const Rssi = Radio868.GetRssi();
    auto const Length = Radio868.ReadPacket( Buffer, sizeof( Buffer ));

    UsbPrintf( "868 Rssi:%4d Snr:%3d.%u Len:%u CRC Error\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
  }

  if( Event == TRadioEvent::NoCrc )
  {
    auto const Snr = Radio868.GetSnr();
    auto const Rssi = Radio868.GetRssi();
    auto const Length = Radio868.ReadPacket( Buffer, sizeof( Buffer ));

    UsbPrintf( "868 Rssi:%4d Snr:%3d.%u Len:%u No CRC\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
  }
}

void TOdie::HAL_GPIO_EXTI_Callback( uint16_t const GPIO_Pin )
{
  switch( GPIO_Pin )
  {
    case DIO0_RADIO433_Pin:
    case DIO1_RADIO433_Pin:
    case DIO2_RADIO433_Pin:
    {
      Radio433Flag = true;
    }
    break;

    case DIO0_RADIO868_Pin:
    case DIO1_RADIO868_Pin:
    case DIO2_RADIO868_Pin:
    {
      Radio868Flag = true;
    }
    break;

    default:
    {
      HmiError();
    }
    break;
  }
}

void TOdie::HmiLoop()
{
  if( TimeoutHmiBlue1 && ( HAL_GetTick() >= TimeoutHmiBlue1 ))
  {
    TimeoutHmiBlue1 = 0;
    ResetPin( LED1_BLUE_GPIO_Port, LED1_BLUE_Pin );
    SetPin( LED1_GREEN_GPIO_Port, LED1_GREEN_Pin );
  }

  if( TimeoutHmiBlue2 && ( HAL_GetTick() >= TimeoutHmiBlue2 ))
  {
    TimeoutHmiBlue2 = 0;
    ResetPin( LED2_BLUE_GPIO_Port, LED2_BLUE_Pin );
    SetPin( LED2_GREEN_GPIO_Port, LED2_GREEN_Pin );
  }
}

void TOdie::Hmi1Blue( uint32_t const Interval )
{
  TimeoutHmiBlue1 = HAL_GetTick() + Interval;
  ResetPin( LED1_GREEN_GPIO_Port, LED1_GREEN_Pin );
  SetPin( LED1_BLUE_GPIO_Port, LED1_BLUE_Pin );
}

void TOdie::Hmi2Blue( uint32_t const Interval )
{
  TimeoutHmiBlue2 = HAL_GetTick() + Interval;
  ResetPin( LED2_GREEN_GPIO_Port, LED2_GREEN_Pin );
  SetPin( LED2_BLUE_GPIO_Port, LED2_BLUE_Pin );
}

void TOdie::HmiError()
{
  Hmi1Error();
  Hmi2Error();
}

void TOdie::Hmi1Error()
{
  ResetPin( LED1_GREEN_GPIO_Port, LED1_GREEN_Pin );
  ResetPin( LED1_BLUE_GPIO_Port, LED1_BLUE_Pin );
  SetPin( LED1_RED_GPIO_Port, LED1_RED_Pin );
}

void TOdie::Hmi2Error()
{
  ResetPin( LED2_GREEN_GPIO_Port, LED2_GREEN_Pin );
  ResetPin( LED2_BLUE_GPIO_Port, LED2_BLUE_Pin );
  SetPin( LED2_RED_GPIO_Port, LED2_RED_Pin );
}

extern "C" void SetupOdie()
{
  Odie.Setup();
}

extern "C" void LoopOdie()
{
  Odie.Loop();
}

extern "C" void HAL_GPIO_EXTI_Callback( uint16_t const GPIO_Pin )
{
  Odie.HAL_GPIO_EXTI_Callback( GPIO_Pin );
}

TOdie::TOdie() :
  Radio433Flag( false ),
  Radio868Flag( false ),
  Radio433(
    433050000U,
    CS_RADIO433_GPIO_Port,
    CS_RADIO433_Pin,
    RESET_RADIO433_GPIO_Port,
    RESET_RADIO433_Pin,
    std::bind( &TOdie::Radio433Event, this, std::placeholders::_1 )),
  Radio868(
    868000000U,
    CS_RADIO868_GPIO_Port,
    CS_RADIO868_Pin,
    RESET_RADIO868_GPIO_Port,
    RESET_RADIO868_Pin,
    std::bind( &TOdie::Radio868Event, this, std::placeholders::_1 )),
  TimeoutHmiBlue1( 0 ),
  TimeoutHmiBlue2( 0 )
{
}
