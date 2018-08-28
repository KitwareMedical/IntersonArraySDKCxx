/*=========================================================================

Library:   IntersonArray

Copyright Kitware Inc. 28 Corporate Drive,
Clifton Park, NY, 12065, USA.

All rights reserved.

Licensed under the Apache License, Version 2.0 ( the "License" );
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

=========================================================================*/
#include <Windows.h> // Sleep
#include <exception>

#include "itkImageFileWriter.h"

#include "IntersonArrayCxxImagingContainer.h"
#include "IntersonArrayCxxControlsHWControls.h"
#include "igtlImageMessage.h"
#include "igtlServerSocket.h"

#include "AcquireIntersonArrayRFCLP.h"

typedef IntersonArrayCxx::Imaging::Container ContainerType;

const unsigned int Dimension = 3;
typedef ContainerType::RFImagePixelType    PixelType;
typedef itk::Image< PixelType, Dimension > ImageType;



struct CallbackClientData
{
  ImageType *        Image;
  itk::SizeValueType FrameIndex;
  igtl::Socket::Pointer socket;
  igtl::ImageMessage * imgMsg;
  igtl::TimeStamp::Pointer ts;
};

void __stdcall pasteIntoImage( PixelType * buffer, void * clientData )
{
  CallbackClientData * callbackClientData =
    static_cast< CallbackClientData * >( clientData );
  ImageType * image =
    callbackClientData->Image;

  const ImageType::RegionType & largestRegion =
    image->GetLargestPossibleRegion();
  const ImageType::SizeType imageSize = largestRegion.GetSize();
  const itk::SizeValueType imageFrames = largestRegion.GetSize()[2];
  

  const int framePixels = imageSize[0] * imageSize[1];

  PixelType * imageBuffer = image->GetPixelContainer()->GetBufferPointer();
  //imageBuffer += framePixels * callbackClientData->FrameIndex;
  //std::cerr << "Attempting copy: buffer to ITK image" << std::endl;
  //std::memcpy( imageBuffer, buffer, framePixels * sizeof( PixelType ) );

  std::cerr << "Acquired frame RF: " << callbackClientData->FrameIndex
    << std::endl;
  ++(callbackClientData->FrameIndex);
  

  callbackClientData->imgMsg->SetMessageID(callbackClientData->FrameIndex);
  callbackClientData->ts->GetTime();
  callbackClientData->imgMsg->SetTimeStamp(callbackClientData->ts);

  try {
    //std::cerr << "Attempting copy: image to message" << std::endl;
    std::memcpy(callbackClientData->imgMsg->GetScalarPointer(), buffer, framePixels * sizeof(PixelType));
  }
  catch (std::exception e)
  {
    std::cerr << "Bad frame! ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    return;
  }
  

  if (callbackClientData->socket.IsNull())
  {
    return;
  }
  //std::cout << "Socket is good! sending" << std::endl;
  callbackClientData->imgMsg->Pack();
  callbackClientData->socket->Send(callbackClientData->imgMsg->GetPackPointer(), callbackClientData->imgMsg->GetPackSize());
}

int main( int argc, char * argv[] )
{
  PARSE_ARGS;

  
  typedef IntersonArrayCxx::Controls::HWControls HWControlsType;
  IntersonArrayCxx::Controls::HWControls hwControls;

  int ret = EXIT_SUCCESS;

  int steering = 0;
  typedef HWControlsType::FoundProbesType FoundProbesType;
  FoundProbesType foundProbes;
  hwControls.FindAllProbes( foundProbes );
  if( foundProbes.empty() )
    {
    std::cerr << "Could not find the probe." << std::endl;
    return EXIT_FAILURE;
    }
  hwControls.FindMyProbe( 0 );
  const unsigned int probeId = hwControls.GetProbeID();
  if( probeId == 0 )
    {
    std::cerr << "Could not find the probe." << std::endl;
    return EXIT_FAILURE;
    }

  HWControlsType::FrequenciesType frequencies;
  hwControls.GetFrequency( frequencies );
  if( !hwControls.SetFrequencyAndFocus( frequencyIndex, focusIndex,
      steering ) )
    {
    std::cerr << "Could not set the frequency and focus." << std::endl;
    return EXIT_FAILURE;
    }

  if( !hwControls.SendHighVoltage( highVoltage, highVoltage ) )
    {
    std::cerr << "Could not set the high voltage." << std::endl;
    return EXIT_FAILURE;
    }
  if( !hwControls.EnableHighVoltage() )
    {
    std::cerr << "Could not enable high voltage." << std::endl;
    return EXIT_FAILURE;
    }

  hwControls.DisableHardButton();

  ContainerType container;
  container.SetHWControls(&hwControls);

  const int height_lines = hwControls.GetLinesPerArray();
  std::cout << "Lines per array: " << height_lines << std::endl;
  const int width_samples = ContainerType::MAX_RFSAMPLES;
  std::cout << "Max RF samples: " << width_samples << std::endl;
  const itk::SizeValueType framesToCollect = 1;

  const double ns = ContainerType::MAX_RFSAMPLES; // number of samples
  const double fs = 30000; // [kHz]=[samples/ms] - sampling frequency
  const double depth = sos * ( ns - 1 ) / ( 2 * fs );
  const double depthCfm = depth/2;
  std::cout << "Depth: " << depth << "mm" << std::endl;
  std::cout << std::endl;

  container.SetRFData( true );

  container.IdleInitScanConverter( depth, width_samples, height_lines, probeId,
      steering, depthCfm, false, false, 0, false );
  container.HardInitScanConverter( depth, width_samples, height_lines, steering,
      depthCfm );

  ImageType::Pointer image = ImageType::New();
  typedef ImageType::RegionType RegionType;
  RegionType imageRegion;
  ImageType::IndexType imageIndex;
  imageIndex.Fill( 0 );
  imageRegion.SetIndex( imageIndex );
  ImageType::SizeType imageSize;
  imageSize[0] = width_samples;
  imageSize[1] = height_lines;
  imageSize[2] = 1;
  imageRegion.SetSize( imageSize );
  image->SetRegions( imageRegion );
  ImageType::SpacingType imageSpacing;
  imageSpacing[ 0 ] = sos / ( 2 * fs );
  imageSpacing[ 1 ] = 38.0 / ( height_lines - 1 );
  const short frameRate = hwControls.GetProbeFrameRate();
  imageSpacing[ 2 ] = 1.0 / frameRate;
  image->SetSpacing( imageSpacing );
  ImageType::DirectionType direction;
  direction.SetIdentity();
  ImageType::DirectionType::InternalMatrixType & vnlDirection = direction.GetVnlMatrix();
  vnlDirection.put(0, 0,  0.0);
  vnlDirection.put(0, 1,  1.0);
  vnlDirection.put(1, 0,  1.0);
  vnlDirection.put(1, 1,  0.0);
  vnlDirection.put(2, 2, -1.0);
  image->SetDirection( direction );
  image->Allocate();

  CallbackClientData clientData;
  clientData.Image = image.GetPointer();
  clientData.FrameIndex = 0;

  // size parameters
  int   size[] = { ContainerType::MAX_RFSAMPLES, ContainerType::NBOFLINES, 1 };       // image dimension
  float spacing[] = { 0.02567, 0.30159, 5.0 };     // spacing (mm/pixel)
  int   svsize[] = { ContainerType::MAX_RFSAMPLES, ContainerType::NBOFLINES, 1 };       // sub-volume size
  int   svoffset[] = { 0, 0, 0 };           // sub-volume offset
  int   scalarType = igtl::ImageMessage::TYPE_INT16;// scalar type

                                                    //------------------------------------------------------------
                                                    // Create a new IMAGE type message
  igtl::ImageMessage::Pointer imgMsg = igtl::ImageMessage::New();
  imgMsg->SetDimensions(size);
  imgMsg->SetSpacing(spacing);
  imgMsg->SetScalarType(scalarType);
  imgMsg->SetDeviceName("ImagerClient");
  imgMsg->SetSubVolume(svsize, svoffset);
  imgMsg->AllocateScalars();
  Sleep(10);
  imgMsg->SetEndian(igtl::ImageMessage::ENDIAN_LITTLE);
  igtl::Matrix4x4 matrix;
  matrix[0][0] = 0.0;  matrix[1][0] = -1.0;  matrix[2][0] = 0.0; matrix[3][0] = 0.0;
  matrix[0][1] = 1.0;  matrix[1][1] = 0.0;  matrix[2][1] = 0.0; matrix[3][1] = 0.0;
  matrix[0][2] = 0.0;  matrix[1][2] = 0.0;  matrix[2][2] = 1.0; matrix[3][2] = 0.0;
  matrix[0][3] = 0.0;  matrix[1][3] = 0.0;  matrix[2][3] = 0.0; matrix[3][3] = 1.0;
  imgMsg->SetMatrix(matrix);
  igtl::TimeStamp::Pointer ts = igtl::TimeStamp::New();
  clientData.ts = ts;
  clientData.imgMsg = imgMsg.GetPointer();

  //socket setup
  int    port = 18944;
  igtl::ServerSocket::Pointer serverSocket;
  serverSocket = igtl::ServerSocket::New();
  int r = serverSocket->CreateServer(port);

  if (r < 0)
  {
    std::cerr << "Cannot create a server socket." << std::endl;
    exit(0);
  }
  std::cerr << "Created Server socket." << std::endl;


  igtl::Socket::Pointer socket;
  clientData.socket = socket;

  container.SetNewRFImageCallback( &pasteIntoImage, &clientData );

  std::cout << "StartRFReadScan" << std::endl;
  container.StartRFReadScan();
  Sleep( 100 ); // "time to start"
  std::cout << "StartRFmode" << std::endl;
  if( !hwControls.StartRFmode() )
    {
    std::cerr << "Could not start RF collection." << std::endl;
    return EXIT_FAILURE;
    }

  int c = 0;
  while( 1 )
    {
    //------------------------------------------------------------
    // Waiting for Connection
    socket = serverSocket->WaitForConnection(1000);
    clientData.socket = socket;
    if (socket.IsNotNull()) // if client connected
    {
      //std::cout << "Socket should be good..." << std::endl;
      
      //recording = true;
      //igtl::Sleep(100);
      Sleep(100);

    }
    }

  hwControls.StopAcquisition();
  container.StopReadScan();
  Sleep( 100 ); // "time to stop"

  

  return ret;
}
