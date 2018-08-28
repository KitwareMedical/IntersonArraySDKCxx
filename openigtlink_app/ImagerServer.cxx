/*=========================================================================

  Program:   OpenIGTLink -- Example for Tracker Client Program
  Language:  C++

  Copyright (c) Insight Software Consortium. All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#include <iostream>
#include <math.h>
#include <cstdlib>
#include <cstdio>

#include "igtlOSUtil.h"
#include "igtlImageMessage.h"
#include "igtlServerSocket.h"

#include "IntersonArrayCxxImagingContainer.h"
#include "IntersonArrayCxxControlsHWControls.h"

typedef IntersonArrayCxx::Imaging::Container ContainerType;
typedef ContainerType::RFImagePixelType    PixelType;
bool recording = false;

struct CallbackClientData
{
  igtl::Socket::Pointer socket;
  int index;
};

void __stdcall pasteIntoImage(PixelType * buffer, void * clientData)
{
  if (!recording)
  {
    return;
  }
  CallbackClientData * callbackClientData =
    static_cast< CallbackClientData * >(clientData);
  

  const int framePixels = ContainerType::MAX_RFSAMPLES * ContainerType::NBOFLINES;
  //------------------------------------------------------------
  // size parameters
  int   size[] = { ContainerType::MAX_RFSAMPLES, ContainerType::NBOFLINES, 1 };       // image dimension
  float spacing[] = { 1.0, 1.0, 5.0 };     // spacing (mm/pixel)
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
  //imgMsg->SetEndian(igtl::ImageMessage::ENDIAN_BIG);  
  std::memcpy(imgMsg->GetScalarPointer(), buffer, framePixels * sizeof(PixelType));
  igtl::Matrix4x4 matrix;
  matrix[0][0] = 1.0;  matrix[1][0] = 0.0;  matrix[2][0] = 0.0; matrix[3][0] = 0.0;
  matrix[0][1] = 0.0;  matrix[1][1] = -1.0;  matrix[2][1] = 0.0; matrix[3][1] = 0.0;
  matrix[0][2] = 0.0;  matrix[1][2] = 0.0;  matrix[2][2] = 1.0; matrix[3][2] = 0.0;
  matrix[0][3] = 0.0;  matrix[1][3] = 0.0;  matrix[2][3] = 0.0; matrix[3][3] = 1.0;
  imgMsg->SetMatrix(matrix);

  //------------------------------------------------------------
  // Pack (serialize) and send
  imgMsg->Pack();
  callbackClientData->socket->Send(imgMsg->GetPackPointer(), imgMsg->GetPackSize());
}



int main(int argc, char* argv[])
{
  //------------------------------------------------------------
  // Parse Arguments

  int    port     = 18944;

  //-------------------------------------------------------------
  //Connect to probe
  typedef IntersonArrayCxx::Controls::HWControls HWControlsType;
  IntersonArrayCxx::Controls::HWControls hwControls;
  int frequencyIndex = 1;
  int focusIndex = 1;
  int highVoltage = 50;
  int sos = 1540;

  int ret = EXIT_SUCCESS;

  int steering = 0;
  typedef HWControlsType::FoundProbesType FoundProbesType;
  FoundProbesType foundProbes;
  hwControls.FindAllProbes(foundProbes);
  if (foundProbes.empty())
  {
    std::cerr << "Could not find the probe." << std::endl;
    return EXIT_FAILURE;
  }
  hwControls.FindMyProbe(0);
  const unsigned int probeId = hwControls.GetProbeID();
  if (probeId == 0)
  {
    std::cerr << "Could not find the probe." << std::endl;
    return EXIT_FAILURE;
  }

  HWControlsType::FrequenciesType frequencies;
  hwControls.GetFrequency(frequencies);
  if (!hwControls.SetFrequencyAndFocus(frequencyIndex, focusIndex,
    steering))
  {
    std::cerr << "Could not set the frequency and focus." << std::endl;
    return EXIT_FAILURE;
  }

  if (!hwControls.SendHighVoltage(highVoltage, highVoltage))
  {
    std::cerr << "Could not set the high voltage." << std::endl;
    return EXIT_FAILURE;
  }
  if (!hwControls.EnableHighVoltage())
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

  const double ns = ContainerType::MAX_RFSAMPLES; // number of samples
  const double fs = 30000; // [kHz]=[samples/ms] - sampling frequency
  const double depth = sos * (ns - 1) / (2 * fs);
  const double depthCfm = depth / 2;
  std::cout << "Depth: " << depth << "mm" << std::endl;
  std::cout << std::endl;

  container.SetRFData(true);

  container.IdleInitScanConverter(depth, width_samples, height_lines, probeId,
    steering, depthCfm, false, false, 0, false);
  container.HardInitScanConverter(depth, width_samples, height_lines, steering,
    depthCfm);

  


  //------------------------------------------------------------
  // Prepare server socket
  igtl::ServerSocket::Pointer serverSocket;
  serverSocket = igtl::ServerSocket::New();
  int r = serverSocket->CreateServer(port);

  if (r < 0)
    {
    std::cerr << "Cannot create a server socket." << std::endl;
    exit(0);
    }


  igtl::Socket::Pointer socket;

  CallbackClientData clientData;
  clientData.socket = socket;

  container.SetNewRFImageCallback(&pasteIntoImage, &clientData);

  std::cout << "StartRFReadScan" << std::endl;
  container.StartRFReadScan();
  Sleep(100); // "time to start"
  std::cout << "StartRFmode" << std::endl;
  if (!hwControls.StartRFmode())
  {
    std::cerr << "Could not start RF collection." << std::endl;
    return EXIT_FAILURE;
  }
  
  while (1)
  {
    //------------------------------------------------------------
    // Waiting for Connection
    socket = serverSocket->WaitForConnection(1000);

    if (socket.IsNotNull()) // if client connected
    {
      recording = true;
      igtl::Sleep(100);
    }
  }
  
  //------------------------------------------------------------
  // Close connection (The example code never reachs to this section ...)
  
  socket->CloseSocket();

}




