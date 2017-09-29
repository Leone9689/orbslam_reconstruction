/*
 * onlinefusionviewer.hpp
 *
 *  Created on: Jun 23, 2013
 *      Author: steinbrf
 */

#ifndef ONLINEFUSIONVIEWER_HPP_
#define ONLINEFUSIONVIEWER_HPP_

#include <QGLViewer/qglviewer.h>
#include <Thirdparty/fastfusion/camerautils/camerautils.hpp>
//#include <fusionGPU/geometryfusion_single_aos.hpp>
#include <Thirdparty/fastfusion/fusion/geometryfusion_mipmap_cpu.hpp>
#include <Thirdparty/fastfusion/fusion/mesh.hpp>
#include <QtCore/QTimer>

#include"KeyFrame.h"
namespace ORB_SLAM2   
{                     
 
class OnlineFusionViewerManipulated : public QGLViewer
{
	Q_OBJECT
public:
	OnlineFusionViewerManipulated(bool createMeshList = true);
	~OnlineFusionViewerManipulated();
  void insertKeyFrame(cv::Mat Tcw , cv::Mat &color, cv::Mat &depth); 
  //void Run();
  std::vector<float> _boundingBox;
  CameraInfo _pose;
  std::vector<std::vector<std::string> > _depthNames;
  std::vector<std::vector<std::string> > _rgbNames;
//  std::vector<Mesh> _meshes;
  MeshSeparate *_currentMeshForSave;
  MeshInterleaved *_currentMeshInterleaved;
  std::vector<PointerMeshDraw*> _pointermeshes;
  FusionMipMapCPU* _fusion;
  float _imageDepthScale;
  float _maxCamDistance;
  long int _currentFrame;
  long int _currentTrajectory;
  long int _firstFrame;
  long int _nextStopFrame;

  bool _threadFusion;
  boost::thread *_fusionThread;
  bool _newMesh;
  bool _fusionActive;
  bool _fusionAlive;

  bool _threadImageReading;

protected slots:
void updateSlot();


protected :
  virtual void init();
  virtual void draw();
  virtual void fastDraw();
  CameraInfo kinectPoseFromMat(cv::Mat &Tcw);
  void drawMeshPointer();
  void drawMeshInterleaved();
  void drawCameraFrustum(CameraInfo &pose,cv::Mat &depthimage,bool showDepthImage);
  virtual void keyPressEvent(QKeyEvent *e);
  void drawGridFrame(float ox, float oy, float oz, float sx, float sy, float sz);
  void setScenePosition(CameraInfo pose);

  void enableLighting();
  void disableLighting();


  long long _lastComputedFrame;

  bool _verbose;
  bool _showCameraFrustum;
  bool _showGridBoundingbox;
  bool _showDepthImage;
  bool _showColor;
  int _displayMode;

  unsigned int _meshNumber;
  unsigned int _fusionNumber;
  unsigned int _keyFrame;
  unsigned int _frame;
  float _cx; float _cy; float _cz;

  std::vector<PointerMeshDraw*> _meshesDraw;
  PointerMeshDraw *_currentMesh;
  unsigned int _currentNV;
  unsigned int _currentNF;
  int _currentMeshType;

  GLuint _vertexBuffer;
  GLuint _faceBuffer;
  GLuint _edgeBuffer;
  GLuint _normalBuffer;
  GLuint _colorBuffer;
  unsigned int _vertexBufferSize;
  unsigned int _faceBufferSize;
  unsigned int _edgeBufferSize;

  bool _onTrack;
  bool _onInterpolation;
  bool _saving;

  qglviewer::KeyFrameInterpolator _interpolator;
  std::vector<qglviewer::ManipulatedFrame*> _keyFrames;

  bool _runFusion;
  bool _createMeshList;
  bool _lightingEnabled;
  bool _colorEnabled;

  bool _lightingInternal;

  KeyFrame* keyframe;     
  cv::Mat   _colorImg;     
  cv::Mat   _depthImg;     
  std::mutex keyframeMutex; 


  QTimer *_timer;

  void generateBuffers();
  bool _buffersGenerated;

};

}

#endif /* ONLINEFUSIONVIEWER_HPP_ */
