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
#ifndef _IntersonArrayCxxImagingContainer_h
#define _IntersonArrayCxxImagingContainer_h

#include "IntersonArrayCxx_Export.h"
#include "IntersonArrayCxxControlsHWControls.h"
#include <string>
#include <vector>

namespace IntersonArrayCxx
{

namespace Imaging
{

class ContainerImpl;

class IntersonArrayCxx_EXPORT Container
{
public:
  Container();
  ~Container();

  typedef unsigned char  PixelType;
  typedef unsigned short RFPixelType;
  typedef short          RFImagePixelType;

  static const int MAX_SAMPLES = 1024;
  static const int MAX_RFSAMPLES = 2048;
  static const int NBOFLINES = 127;

  bool GetCompound();

  void SetCompound( bool value );

  bool GetDoubler();

  void SetDoubler( bool value );

  int GetHeightScan() const;

  float GetMmPerPixel() const;

  double GetTrueDepth() const;

  int GetWidthScan() const;

  int GetZeroOfYScale() const;

  // Error code of HardInitScanConverter and SoftInitScanConverter
  enum ScanConverterError
  {
    SUCCESS = 1,
    PROBE_NOT_INITIALIZED, // FindMyProbe has not been called
    UNKNOWN_PROBE, // Probe Identity not valid
    UNDER_LIMITS, // Width * Height over 10 * 10
    OVER_LIMITS, // Width * Height over 800 * 1000
    WRONG_FORMAT, // other error
    OTHER_ERROR
  };

  //widthScan and heightScan are the size of the final converted image
  ScanConverterError HardInitScanConverter( int depth, int widthScan,
    int heightScan, int steering, int depthCfm );

  ScanConverterError IdleInitScanConverter( int depth, int width,
    int height, short idleId, int idleSteering, int depthCfm, bool idleDoubler,
    bool idleCompound, int idleCompoundAngle, bool idleCfm );
  
  //
  // Begin Capture Methods
  //
  bool GetRFData();

  void SetRFData( bool value );

  bool GetFrameAvg();

  void SetFrameAvg( bool value );

  bool GetScanOn() const;

  void StartReadScan();

  void StartRFReadScan();

  void StopReadScan();

  typedef void (__stdcall *NewImageCallbackType)( PixelType *buffer,
    void *clientData );
  void SetNewImageCallback( NewImageCallbackType callback,
    void *clientData = NULL );

  typedef void (__stdcall *NewRFImageCallbackType)( RFImagePixelType *buffer,
    void *clientData );
  void SetNewRFImageCallback( NewRFImageCallbackType callback,
    void *clientData = NULL );

  void SetHWControls(IntersonArrayCxx::Controls::HWControls * controls);

  //Converts an array of b-mode transducer responses into an undistorted
  //2-D image of size widthScan, heightScan.
  void Build2D(PixelType * bmode_bytedata, PixelType * image_out);

private:

  Container( const Container &);
  void operator=( const Container &);

  ContainerImpl *Impl;
};

} // end namespace Imaging

} // end namespace IntersonArrayCxx


#endif // _IntersonArrayCxxImagingContainer_h
