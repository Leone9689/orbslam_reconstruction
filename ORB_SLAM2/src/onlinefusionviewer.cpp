/*
 * onlinefusionviewer.cpp
 *
 *  Created on: Jun 23, 2013
 *      Author: steinbrf
 */
#include <GL/glew.h>
#include "onlinefusionviewer.hpp"
#include <math.h>
#include <QMenu>
#include <QKeyEvent>
#include <QMouseEvent>

#include <Eigen/Geometry>
#include <stdio.h>
#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "Camera.h"
#include "Converter.h"
//#define PREPROCESS_IMAGES

//#define DEBUG_NO_MESHES
//#define DEBUG_NO_MESH_VISUALIZATION

namespace ORB_SLAM2
{

#include <opencv2/opencv.hpp>

OnlineFusionViewerManipulated::OnlineFusionViewerManipulated(bool createMeshList) :QGLViewer(),
_currentMeshForSave(NULL),
_currentMeshInterleaved(NULL),
	_fusion(NULL),
	_imageDepthScale(5000.0), _maxCamDistance(MAXCAMDISTANCE),
	_currentFrame(-1),
	_currentTrajectory(-1),
//	_firstFrame(-1),
	_nextStopFrame(0),
	_threadFusion(false),
	_fusionThread(NULL),
	_newMesh(false),
	_fusionActive(true),
	_fusionAlive(true),
	_threadImageReading(false),
	_lastComputedFrame(-1),
		_verbose(true),
		_showCameraFrustum(false), _showDepthImage(true),
		_currentMesh(NULL),_currentNV(0),_currentNF(0), _currentMeshType(0),
		_vertexBuffer(0), _faceBuffer(0), _edgeBuffer(0), _colorBuffer(0),
		_vertexBufferSize(0), _faceBufferSize(0), _edgeBufferSize(0),
		_onTrack(false), _onInterpolation(false), _saving(false),
		 _runFusion(false), _createMeshList(createMeshList)
,_lightingEnabled(false)
,_colorEnabled(true)
,_lightingInternal(false)
, _timer(NULL)
,_buffersGenerated(false)
{
 

  _meshNumber = 0;
	_fusionNumber = 0;
  _keyFrame = 0;
  _frame = 0;
	_cx = 0.0f; _cy = 0.0f; _cz = 0.0f;

	_timer = new QTimer(this);
	_timer->start();

}

OnlineFusionViewerManipulated::~OnlineFusionViewerManipulated()
{
	_timer->stop();
	delete _timer;
	_fusionAlive = false;

	if(_buffersGenerated){
		glDeleteBuffers(1,&_vertexBuffer);
		glDeleteBuffers(1,&_faceBuffer);
		glDeleteBuffers(1,&_colorBuffer);
		glDeleteBuffers(1,&_normalBuffer);
	}

	if(_fusionThread){
		_fusionThread->join();
		delete _fusionThread;
	}
	if(_fusion) delete _fusion;
	if(_currentMeshForSave) delete _currentMeshForSave;
	if(_currentMeshInterleaved) delete _currentMeshInterleaved;

	fprintf(stderr,"\nEnd of OnlineFusionViewerManipulated Destructor.");
#ifdef INTERACTIVE_MEMORY_MANAGEMENT
	if(INTERACTIVE_MEMORY_MANAGEMENT){
		fprintf(stderr,"\nInput:");
		char input[256];
		fprintf(stderr,"%s",fgets(input,256,stdin));
	}
#endif
}

CameraInfo OnlineFusionViewerManipulated::kinectPoseFromMat( cv::Mat &Tcw)
{
	CameraInfo result;
  cv::Mat intrinsic = cv::Mat::eye(3,3,cv::DataType<double>::type); 
  /*float fx = 986.25f;                   
  float fy = 981.41f;                   
  float cx = 1005.3f;                   
  float cy = 677.09f;   
  */
  /*float fx = 525.0f;
  float fy = 525.0f;
  float cx = 319.5f;
  float cy = 239.5f;
  */
  intrinsic.at<double>(0,0) = Camera::fx;      
  intrinsic.at<double>(1,1) = Camera::fy;      
  intrinsic.at<double>(0,2) = Camera::cx;      
  intrinsic.at<double>(1,2) = Camera::cy;      
                                       
  result.setIntrinsic(intrinsic);      

  Eigen::Matrix<double,3,3> eigMat =ORB_SLAM2::Converter::toMatrix3d(Tcw);   
  Eigen::Quaterniond q(eigMat);                                              
/* 
  cv::Mat tcw = cv::Mat::zeros(3,1,cv::DataType<double>::type);

  cv::Mat tcwMat = Tcw.rowRange(0,3).col(3);        
  tcw.at<double>(0,0) = tcwMat.at<double>(0);             
  tcw.at<double>(1,0) = tcwMat.at<double>(1);             
  tcw.at<double>(2,0) = tcwMat.at<double>(2);             

  Eigen::Matrix3d R = Eigen::Quaterniond(q.w(),q.x(),q.y(),q.z()).toRotationMatrix() ;             
  Eigen::Matrix3d R1 = Eigen::Quaterniond(q.w(),-q.x(),-q.y(),-q.z()).toRotationMatrix() ;         
                                                                                                      
  cv::Mat Rcw = cv::Mat::eye(3,3,cv::DataType<double>::type);                                      
  for(int i=0;i<3;i++) {                                                                           
    for(int j=0;j<3;j++) {                                                                         
      Rcw.at<double>(i,j) = R(i,j);                                                                
    }                                                                                              
  }                                                                                                
  cv::Mat Rwc = Rcw.t();                                                                           
  cv::Mat Rcw1 = cv::Mat::eye(3,3,cv::DataType<double>::type);                                     
  for(int i=0;i<3;i++) {                                                                           
    for(int j=0;j<3;j++) {                                                                         
      Rcw1.at<double>(i,j) = R1(i,j);                                                              
    }                                                                                              
  }                                                                                                
  cv::Mat Ow = cv::Mat::zeros(3,1,cv::DataType<double>::type);
  Ow = -Rwc*tcw;                                                                           
  */ 
  //cv::Mat Twc = Tcw.t();
  cv::Mat tcw = Tcw.rowRange(0,3).col(3);
  cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3); 
  cv::Mat Rwc = Rcw.t();
  //vector<float> q = Converter::toQuaternion(Rwc);
  cv::Mat Ow = -Rwc*tcw;                                  
  Eigen::Matrix3d rotation = Eigen::Quaterniond(q.w(),-q.x(),-q.y(),-q.z()).toRotationMatrix();
  
  cv::Mat rotation2 = cv::Mat::eye(3,3,cv::DataType<double>::type);
  for(int i=0;i<3;i++) 
    for(int j=0;j<3;j++) 
      rotation2.at<double>(i,j) = rotation(i,j);
  result.setRotation(rotation2);
  cv::Mat translation2 = cv::Mat::zeros(3,1,cv::DataType<double>::type);
  for(int i=0;i<3;i++) 
    translation2.at<double>(i,0) = Ow.at<float>(i);
  result.setTranslation(translation2);

  //std::cout<<"q[1]:"<<q.x()<<",q[2]:"<<q.y()<<",q[3]:"<<q.z()<<",q[4]:"<<q.w()<<std::endl;
  //std::cout<<"TCW:"<<translation2<<std::endl;
  //std::cout<<"Ow[0]:"<<Ow.at<float>(0)<<",Ow[1]:"<<Ow.at<float>(1)<<",Ow[2]:"<<Ow.at<float>(2)<<std::endl;
  return result;
}
void OnlineFusionViewerManipulated::insertKeyFrame(cv::Mat Tcw , cv::Mat &color, cv::Mat &depth) 
{
   unique_lock<mutex> lock(keyframeMutex);              
   _pose = kinectPoseFromMat(Tcw);
   _colorImg = color.clone();                
   _depthImg = depth.clone();
   _keyFrame++;
   std::cout<<"keyframe id"<<_keyFrame<<std::endl;   
}

void OnlineFusionViewerManipulated::enableLighting(){_lightingInternal=true;   glColor3f(0.5,0.5,0.5); glEnable(GL_LIGHTING);}
void OnlineFusionViewerManipulated::disableLighting(){_lightingInternal=false; glDisable(GL_LIGHTING);}

void OnlineFusionViewerManipulated::drawGridFrame(float ox, float oy, float oz, float sx, float sy, float sz)
{
	disableLighting();
	glColor3f(0.0f,1.0f,0.0f);
	glBegin(GL_LINES);
	glVertex3f(ox,oy,oz);glVertex3f(ox+sx,oy,oz);
	glVertex3f(ox,oy+sy,oz);glVertex3f(ox+sx,oy+sy,oz);
	glVertex3f(ox,oy,oz+sz);glVertex3f(ox+sx,oy,oz+sz);
	glVertex3f(ox,oy+sy,oz+sz);glVertex3f(ox+sx,oy+sy,oz+sz);

	glVertex3f(ox,oy,oz);glVertex3f(ox,oy+sy,oz);
	glVertex3f(ox+sx,oy,oz);glVertex3f(ox+sx,oy+sy,oz);
	glVertex3f(ox,oy,oz+sz);glVertex3f(ox,oy+sy, oz+sz);
	glVertex3f(ox+sx,oy,oz+sz);glVertex3f(ox+sx,oy+sy,oz+sz);

	glVertex3f(ox,oy,oz);glVertex3f(ox,oy,oz+sz);
	glVertex3f(ox+sx,oy,oz);glVertex3f(ox+sx,oy,oz+sz);
	glVertex3f(ox,oy+sy,oz);glVertex3f(ox,oy+sy,oz+sz);
	glVertex3f(ox+sx,oy+sy,oz);glVertex3f(ox+sx,oy+sy,oz+sz);
	glEnd();
	enableLighting();

}

void OnlineFusionViewerManipulated::generateBuffers(){
	glewInit();
	glGenBuffers(1, &_vertexBuffer);
	glGenBuffers(1, &_faceBuffer);
	glGenBuffers(1, &_colorBuffer);
	glGenBuffers(1, &_normalBuffer);
	_buffersGenerated = true;
}


void OnlineFusionViewerManipulated::drawMeshPointer()
{
	fprintf(stderr,"\nDrawing Pointer Mesh");

  glColor3f(1,1,1);

	long int drawFrame = _createMeshList ? _currentFrame : 0;
	if(drawFrame<(long int)_pointermeshes.size() && _pointermeshes[drawFrame] &&_pointermeshes[drawFrame]->nf){
		if(_pointermeshes[drawFrame] != _currentMesh
				|| _pointermeshes[drawFrame]->type != _currentMeshType
				|| _pointermeshes[drawFrame]->nv != _currentNV
				|| _pointermeshes[drawFrame]->nf != _currentNF
				){
			if(!_buffersGenerated){
				generateBuffers();
			}
			_currentMesh = _pointermeshes[drawFrame];
			_currentNV = _currentMesh->nv;
			_currentNF = _currentMesh->nf;
			_currentMeshType = _currentMesh->type;

			glBindBuffer(GL_ARRAY_BUFFER,_vertexBuffer);
			glBufferData(GL_ARRAY_BUFFER,_currentMesh->nv*3*sizeof(float),_currentMesh->v, GL_STATIC_DRAW);
			if(_currentMesh->colored){
				glBindBuffer(GL_ARRAY_BUFFER,_colorBuffer);
				glBufferData(GL_ARRAY_BUFFER,_currentMesh->nv*3,_currentMesh->c, GL_STATIC_DRAW);
			}
			if(_currentMesh->type==1){
				glBindBuffer(GL_ARRAY_BUFFER,_normalBuffer);
				glBufferData(GL_ARRAY_BUFFER,_currentMesh->nn*3*sizeof(float),_currentMesh->n, GL_STATIC_DRAW);
			}
			if(_currentMesh->type != 1 && _currentMesh->nf){
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,_faceBuffer);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, _currentMesh->nf*sizeof(unsigned int),
						_currentMesh->f, GL_STATIC_DRAW);
			}
		}
//		fprintf(stderr,"\nChecking Mesh");
//		for(unsigned int f=0;f<_currentMesh->nf;f++){
//			fprintf(stderr," %i",_currentMesh->f[f]);
//			if(_currentMesh->f[f]>=_currentMesh->nv){
//				fprintf(stderr,"\nERROR: [%i %i]",_currentMesh->f[f],_currentMesh->nv);
//			}
//		}
//		fprintf(stderr,"\nMesh Check done");
//		if(_lightingEnabled) {
//			if(_currentFrame >=_firstFrame && _currentFrame <= _nextStopFrame){
//				cv::Mat trans = _poses[_currentTrajectory][_currentFrame].getTranslation();
//				enableLighting();
////				glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
//			  GLfloat mat_amb_diff[] = { 1.0, 0.1, 0.1, 1.0 };
//			  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat_amb_diff);
//
//			  float lightDiff[] = {1.0, 0.0, 1.0};
//				glLightfv(GL_LIGHT4, GL_DIFFUSE, lightDiff);
//				glLightfv(GL_LIGHT4, GL_SPECULAR, lightDiff);
//				float lightPoseCamera[3] = {trans.at<double>(0,0),trans.at<double>(1,0),trans.at<double>(2,0)};
//				glLightfv(GL_LIGHT4, GL_POSITION, lightPoseCamera);
//				glEnable(GL_LIGHT4);
//				glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,0.5f);
//			}
//		}

		glBindBuffer(GL_ARRAY_BUFFER,_vertexBuffer);
		glVertexPointer(3, GL_FLOAT, 0, 0);
		if(_currentMesh->colored){
			glBindBuffer(GL_ARRAY_BUFFER,_colorBuffer);
			glColorPointer(3, GL_UNSIGNED_BYTE, 0, 0);
		}
		else{
			glColor3f(0.5f,0.5f,0.5f);
		}
		if(_currentMesh->type==1 && _currentMesh->nn>0){
			glBindBuffer(GL_ARRAY_BUFFER,_normalBuffer);
			glNormalPointer(GL_FLOAT, 0,0);
		}

		glEnableClientState(GL_VERTEX_ARRAY);
		if(_colorEnabled) {
			glEnableClientState(GL_COLOR_ARRAY);
		}
		else{
			glColor3f(0.5f,0.5f,0.5f);
		}
		if(_currentMesh->type==1 && _currentMesh->nn>0){
			if(_colorEnabled) glEnableClientState(GL_NORMAL_ARRAY);
		}

		if(_displayMode==1){
			glPolygonMode(GL_FRONT, GL_LINE);
			glPolygonMode(GL_BACK, GL_LINE);
			glLineWidth(0.5f);
		}
		else{
			glPolygonMode(GL_FRONT, GL_FILL);
			glPolygonMode(GL_BACK, GL_FILL);
		}


		if(_currentMesh->type==1){
			if(_displayMode==2){
				fprintf(stderr,"\nDrawing Points");
				glDrawArrays(GL_POINTS, 0,_currentMesh->nv);
			}
			else{
				fprintf(stderr,"\nDrawing Triangles");
				glDrawArrays(GL_TRIANGLES, 0,_currentMesh->nv);
			}
		}
		else{
			if(_displayMode==2){
				fprintf(stderr,"\nDrawing Points");
				glDrawArrays(GL_POINTS, 0,_currentMesh->nv);
			}
			else{
				fprintf(stderr,"\nDrawing Triangles");
				glDrawElements(GL_TRIANGLES, _currentMesh->nf, GL_UNSIGNED_INT,0);
			}
		}
		if(_colorEnabled) glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);
			}
	fprintf(stderr,"\nPointer Mesh drawn");
}

void OnlineFusionViewerManipulated::drawMeshInterleaved()
{
//	fprintf(stderr,"\nDrawing interleaved Mesh");

  glColor3f(1,1,1);

	if(_currentMeshInterleaved && _currentMeshInterleaved->faces.size()){
		if(_currentMeshInterleaved->vertices.size() != _currentNV ||
				_currentMeshInterleaved->faces.size() != _currentNF){
			eprintf("\nReassigning Buffers for interleaved Mesh");
			if(!_buffersGenerated){
				generateBuffers();
			}
			_currentNV = _currentMeshInterleaved->vertices.size();
			_currentNF = _currentMeshInterleaved->faces.size();

			glBindBuffer(GL_ARRAY_BUFFER,_vertexBuffer);
			glBufferData(GL_ARRAY_BUFFER,_currentMeshInterleaved->vertices.size()*3*sizeof(float),_currentMeshInterleaved->vertices.data(), GL_STATIC_DRAW);
			if(_currentMeshInterleaved->colors.size()){
				glBindBuffer(GL_ARRAY_BUFFER,_colorBuffer);
				glBufferData(GL_ARRAY_BUFFER,_currentMeshInterleaved->colors.size()*3,_currentMeshInterleaved->colors.data(), GL_STATIC_DRAW);
			}

			if(_currentMeshInterleaved->faces.size()){
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,_faceBuffer);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, _currentMeshInterleaved->faces.size()*sizeof(unsigned int),
						_currentMeshInterleaved->faces.data(), GL_STATIC_DRAW);
			}

			eprintf("\nChecking Mesh...");
			std::vector<bool> checks(_currentMeshInterleaved->vertices.size(),false);
			for(size_t i=0;i<_currentMeshInterleaved->faces.size();i++)
				checks[_currentMeshInterleaved->faces[i]] = true;

			bool loneVertex = false;
			for(size_t i=0;i<checks.size();i++) loneVertex |= !checks[i];
			if(loneVertex){
				fprintf(stderr,"\nThere were lone Vertices!");
			}
			eprintf("\nMesh Check done");
		}


		glBindBuffer(GL_ARRAY_BUFFER,_vertexBuffer);
		glVertexPointer(3, GL_FLOAT, 0, 0);
		if(_currentMeshInterleaved->colors.size()){
			glBindBuffer(GL_ARRAY_BUFFER,_colorBuffer);
			glColorPointer(3, GL_UNSIGNED_BYTE, 0, 0);
		}
		else{
			glColor3f(0.5f,0.5f,0.5f);
		}

		glEnableClientState(GL_VERTEX_ARRAY);
		if(_colorEnabled) {
			glEnableClientState(GL_COLOR_ARRAY);
		}
		else{
			glColor3f(0.5f,0.5f,0.5f);
		}


		if(_displayMode==1){
			glPolygonMode(GL_FRONT, GL_LINE);
			glPolygonMode(GL_BACK, GL_LINE);
			glLineWidth(0.5f);
		}
		else{
			glPolygonMode(GL_FRONT, GL_FILL);
			glPolygonMode(GL_BACK, GL_FILL);
		}

		if(_displayMode==2){
			glPointSize(2.0);
			glBindBuffer(GL_ARRAY_BUFFER,_vertexBuffer);
			glDrawArrays(GL_POINTS,0,_currentMeshInterleaved->vertices.size());
		}
		else{
			glDrawElements(GL_TRIANGLES, _currentMeshInterleaved->faces.size(), GL_UNSIGNED_INT,0);
		}

		if(_colorEnabled) glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);
	}
}




void OnlineFusionViewerManipulated::drawCameraFrustum
(
		CameraInfo &pose,
		cv::Mat &depthimage,
    bool showDepthImage
)
{
//	fprintf(stderr,"\nDrawing Camera Frustum");
  cv::Mat intr = pose.getIntrinsic();
  cv::Mat ext = pose.getExtrinsic().clone();

  double fx = intr.at<double>(0,0);
  double fy = intr.at<double>(1,1);
  double cx = intr.at<double>(0,2);
  double cy = intr.at<double>(1,2);

  double tx = ext.at<double>(0,3)-_cx;
  double ty = ext.at<double>(1,3)-_cy;
  double tz = ext.at<double>(2,3)-_cz;

  double r11 = ext.at<double>(0,0);
  double r12 = ext.at<double>(0,1);
  double r13 = ext.at<double>(0,2);
  double r21 = ext.at<double>(1,0);
  double r22 = ext.at<double>(1,1);
  double r23 = ext.at<double>(1,2);
  double r31 = ext.at<double>(2,0);
  double r32 = ext.at<double>(2,1);
  double r33 = ext.at<double>(2,2);


  bool lightingwasenabled = _lightingInternal;
  disableLighting();

  glLineWidth(2.0);
  glColor3f(1,0,0);
  glBegin(GL_LINES);

//  double px = (0.0 - cx); double py = (0.0 - cy); double pz = fx;
  double px = (0.0 - cx)/fx; double py = (0.0 - cy)/fx; double pz = 1.0f;
  double qx = r11*px + r12*py + r13*pz;
  double qy = r21*px + r22*py + r23*pz;
  double qz = r31*px + r32*py + r33*pz;
  glVertex3f(tx,ty,tz);
  glVertex3f(qx+tx,qy+ty,qz+tz);

//  px = (640.0 - cx); py = (0.0f - cy); pz = fx;
  px = (640.0 - cx)/fx; py = (0.0f - cy)/fx; pz = 1.0f;
  qx = r11*px + r12*py + r13*pz;
  qy = r21*px + r22*py + r23*pz;
  qz = r31*px + r32*py + r33*pz;
  glVertex3f(tx,ty,tz);
  glVertex3f(qx+tx,qy+ty,qz+tz);

//  px = (640.0 - cx); py = (480.0 - cy); pz = fx;
  px = (640.0 - cx)/fx; py = (480.0 - cy)/fx; pz = 1.0f;
  qx = r11*px + r12*py + r13*pz;
  qy = r21*px + r22*py + r23*pz;
  qz = r31*px + r32*py + r33*pz;
  glVertex3f(tx,ty,tz);
  glVertex3f(qx+tx,qy+ty,qz+tz);

//  px = (0.0 - cx); py = (480.0 - cy); pz = fx;
  px = (0.0 - cx)/fx; py = (480.0 - cy)/fx; pz = 1.0f;
  qx = r11*px + r12*py + r13*pz;
  qy = r21*px + r22*py + r23*pz;
  qz = r31*px + r32*py + r33*pz;
  glVertex3f(tx,ty,tz);
  glVertex3f(qx+tx,qy+ty,qz+tz);
  glEnd();

  if(lightingwasenabled) enableLighting();


	if(showDepthImage){
		if(depthimage.empty()) fprintf(stderr,"\nERROR: DepthIMage empty");
		glPointSize(2.0);
	  glColor3f(1,0,0);
		glBegin(GL_POINTS);
		for(int x=0;x<depthimage.cols;x++){
			for(int y=0;y<depthimage.rows;y++){
				float h = depthimage.at<float>(y,x);
				if(h > 0.0f){
					px = ((float)x-cx)/fx*h; py = ((float)y-cy)/fy*h; pz = h;
					qx = r11*px + r12*py + r13*pz + tx;
					qy = r21*px + r22*py + r23*pz + ty;
					qz = r31*px + r32*py + r33*pz + tz;
					glVertex3f(qx,qy,qz);
				}
			}
		}
		glEnd();
	}

}

void OnlineFusionViewerManipulated::init()
{

  setShortcut(EXIT_VIEWER, Qt::CTRL+Qt::Key_Q);
  setHandlerKeyboardModifiers(QGLViewer::CAMERA, Qt::AltModifier);
  setHandlerKeyboardModifiers(QGLViewer::FRAME,  Qt::NoModifier);
  setHandlerKeyboardModifiers(QGLViewer::CAMERA, Qt::ControlModifier);

  setSnapshotFormat("PNG");
  setSnapshotCounter(0);


#ifdef GL_RESCALE_NORMAL  // OpenGL 1.2 Only...
  glEnable(GL_RESCALE_NORMAL);
#endif

  setManipulatedFrame(new qglviewer::ManipulatedFrame());

  restoreStateFromFile();

//  GLfloat mat_amb_diff[] = { 1.0, 0.1, 0.1, 1.0 };
//  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat_amb_diff);
////  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
//
//  float lightDiff[] = {1., 1., 1.};
//  endableLighting();
//  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiff);
//  glLightfv(GL_LIGHT0, GL_SPECULAR, lightDiff);
//  glLightfv(GL_LIGHT1, GL_DIFFUSE, lightDiff);
//  glLightfv(GL_LIGHT1, GL_SPECULAR, lightDiff);
//  float lightPos0[] = {-0.1, -0.1, 0.1};
//  float lightPos1[] = {0., 0., 0.};
//  glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
//  glLightfv(GL_LIGHT1, GL_POSITION, lightPos1);
//  glEnable(GL_LIGHT0);
//  glEnable(GL_LIGHT1);
//  glShadeModel(GL_SMOOTH);

//  setBackgroundColor(QColor(255,255,255,255));


  double m[3][3];
  m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f;
  m[1][0] = 0.0f; m[1][1] = -1.0f; m[1][2] = 0.0f;
  m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = -1.0f;
  qglviewer::Quaternion orientation;
  orientation.setFromRotationMatrix(m);

  camera()->setPosition(qglviewer::Vec(0.0f,0.0f,-0.0f));
  camera()->setOrientation(orientation);



  // Make world axis visible
//  setAxisIsDrawn();

  _showGridBoundingbox = false;
  _showDepthImage = false;
  _showColor = true;
  _displayMode = false;


	camera()->setFieldOfView(2.0f*atan(240.0/525.0));
//	resize(640,480);

	if(_lightingEnabled) enableLighting();
	else                 disableLighting();

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

}

void OnlineFusionViewerManipulated::setScenePosition(CameraInfo pose)
{
	CameraInfo inv = pose;
	inv.setExtrinsic(pose.getExtrinsicInverse());
  cv::Mat ext = inv.getExtrinsic().clone();
  double tx = ext.at<double>(0,3)+_cx;
  double ty = ext.at<double>(1,3)+_cy;
  double tz = ext.at<double>(2,3)+_cz;
//  cv::Mat quat = pose.getQuaternion();

  double m[3][3];
  for(int i=0;i<3;i++) for(int j=0;j<3;j++) m[i][j] = inv.getRotation().at<double>(i,j);
  qglviewer::Quaternion orientation;
  orientation.setFromRotationMatrix(m);
	manipulatedFrame()->setPosition(tx,ty,tz);
	manipulatedFrame()->setOrientation(orientation);




//	manipulatedFrame()->setOrientation(quat.at<double>(3),quat.at<double>(2),quat.at<double>(1),quat.at<double>(0));
}

void OnlineFusionViewerManipulated::draw()
{

  CameraInfo pose;
  cv::Mat depthimage;
  { 
    unique_lock<mutex> lock(keyframeMutex);    
    depthimage = _depthImg;
    pose = _pose ;   
  }
	glMatrixMode(GL_PROJECTION);
//	glFrustum(-1.0,1.0,-1.0,1.0,0.0001,1000.0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	camera()->setZNearCoefficient (0.0001);
	camera()->setZClippingCoefficient (100.0);


  glMultMatrixd(manipulatedFrame()->matrix());

  glScalef(1.0f, 1.0f, 1.0f);


  if(_lightingEnabled){
  	drawMeshPointer();
  }
  else{
    drawMeshInterleaved();
  }
  	drawCameraFrustum(pose,depthimage,_showDepthImage);
  if(_showGridBoundingbox && _boundingBox.size()==6)
  	drawGridFrame(_boundingBox[0]-_cx,_boundingBox[1]-_cy,_boundingBox[2]-_cz,
  			          _boundingBox[3]-_cx,_boundingBox[4]-_cy,_boundingBox[5]-_cz);

  // Restore the original (world) coordinate system
  glPopMatrix();
}

void OnlineFusionViewerManipulated::fastDraw()
{
  // Save the current model view matrix (not needed here in fact)
  glPushMatrix();

  // Multiply matrix to get in the frame coordinate system.
  glMultMatrixd(manipulatedFrame()->matrix());

  glScalef(1.0f, 1.0f, 1.0f);

//   Draw an axis using the QGLViewer static function
  drawAxis();

  if(_showGridBoundingbox && _boundingBox.size()==6)
  	drawGridFrame(_boundingBox[0]-_cx,_boundingBox[1]-_cy,_boundingBox[2]-_cz,
  			          _boundingBox[3]-_cx,_boundingBox[4]-_cy,_boundingBox[5]-_cz);

  // Restore the original (world) coordinate system
  glPopMatrix();
}


void OnlineFusionViewerManipulated::keyPressEvent(QKeyEvent *e)
{
  // Get event modifiers key
#if QT_VERSION < 0x040000
  // Bug in Qt : use 0x0f00 instead of Qt::KeyButtonMask with Qt versions < 3.1
  const Qt::ButtonState modifiers = (Qt::ButtonState)(e->state() & Qt::KeyButtonMask);
#else
  const Qt::KeyboardModifiers modifiers = e->modifiers();
#endif


  bool handled = false;
  if ((e->key()==Qt::Key_S) && (modifiers==Qt::NoButton))
  {
  	if(_verbose) fprintf(stderr,"\n\nSwitching Fusion %s\n",_runFusion ? "off" : "on");
  	_runFusion = !_runFusion;
  	if(_runFusion){
  		connect(_timer,SIGNAL(timeout()),this,SLOT(updateSlot()));
  		_fusionActive = true;
  	}
  	else{
  		disconnect(_timer,SIGNAL(timeout()),this,SLOT(updateSlot()));
  		_fusionActive = false;
  	}
  	handled = true;
//  	updateGL();
  }
  else if((e->key()==Qt::Key_N) && (modifiers==Qt::NoButton)){
  	_runFusion = false;
  	_nextStopFrame = _currentFrame;
  	handled = true;
  	updateGL();
  }
  else if((e->key()==Qt::Key_G) && (modifiers==Qt::NoButton)){
  	if(_runFusion || _fusionActive){
  		fprintf(stderr,"\nYou have to stop the fusion first for single step!");
  	}
  	else{
  		fprintf(stderr,"\nSingle Frame Update");
  		_runFusion = true; _fusionActive = true;
  		updateSlot();
  		_runFusion = false; _fusionActive = false;
    	handled = true;
    	updateGL();
  	}
  }
  else if((e->key()==Qt::Key_Left) && (modifiers==Qt::NoButton)){
  	if(_currentFrame > _firstFrame) _currentFrame--;
  	fprintf(stderr,"\n Frame %li of [%li %lli]",_currentFrame,_firstFrame,_lastComputedFrame);
  	handled = true;
  	updateGL();
  }
  else if((e->key()==Qt::Key_Right) && (modifiers==Qt::NoButton)){
  	if(_currentFrame<_lastComputedFrame) _currentFrame++;
  	fprintf(stderr,"\n Frame %li of [%li %lli]",_currentFrame,_firstFrame,_lastComputedFrame);
  	handled = true;
  	updateGL();
  }
  else if((e->key()==Qt::Key_W) && (modifiers==Qt::NoButton)){
  	if(_currentMeshForSave){
  		fprintf(stderr,"\nWriting Current Mesh...");
//  		_currentMeshForSave->writePLY("current");
  		_currentMeshInterleaved->writePLY("current",false);
  		fprintf(stderr,"\nCurrent Mesh written");
  	}
  	handled = true;
  }
  else if((e->key()==Qt::Key_F) && (modifiers==Qt::NoButton)){
  	fprintf(stderr,"\nSaving Snapshot...");
  	saveSnapshot(false);
  	fprintf(stderr,"done");
  }
  else if((e->key()==Qt::Key_O) && (modifiers==Qt::NoButton)){
  	int width = 640, height = 480;
  	std::cerr << "\nResizing\nEnter Width:\n";
  	std::cin >> width;
  	std::cerr << "\nEnter Height:\n";
  	std::cin >> height;
  	resize(width,height);
  }
  else if((e->key()==Qt::Key_L) && (modifiers==Qt::NoButton)){
  	_lightingEnabled = !_lightingEnabled;
  	fprintf(stderr,"\nLighting %s",_lightingEnabled ? "enabled" : "disabled");
  	long int drawFrame = _createMeshList ? _currentFrame : 0;
  	if(_lightingEnabled){
  		enableLighting();
    	if( _pointermeshes.size() &&
    			_pointermeshes[drawFrame] && _pointermeshes[drawFrame]->type != 1){
    		fprintf(stderr,"\nCreating Non-indexed PointerMeshDraw");
    		PointerMeshDraw *oldMesh = _pointermeshes[drawFrame];
    		_pointermeshes[drawFrame] = new PointerMeshDraw(*oldMesh,1);
    		delete oldMesh;
    		fprintf(stderr,"\nNon-indexed PointerMeshDraw created");
    	}
  	}
  	else{
  		disableLighting();
  	}

  	handled = true;
  	updateGL();
  }
  else if((e->key()==Qt::Key_C) && (modifiers==Qt::NoButton)){
  	_colorEnabled = !_colorEnabled;
  	fprintf(stderr,"\nColor %s",_colorEnabled ? "enabled" : "disabled");
  	handled = true;
  	updateGL();
  }
  else if((e->key()==Qt::Key_K) && (modifiers==Qt::NoButton)){
  	_showDepthImage = !_showDepthImage;
  	fprintf(stderr,"\nShowing Depth Image %s",_showDepthImage ? "enabled" : "disabled");
  	handled = true;
  	updateGL();
  }
  else if((e->key()==Qt::Key_P) && (modifiers==Qt::NoButton)){
  	_displayMode = (_displayMode+1)%3;
  	if(_displayMode==0) fprintf(stderr,"\nDisplay Mode is Faces");
  	if(_displayMode==1) fprintf(stderr,"\nDisplay Mode is WireFrame");
  	if(_displayMode==2) fprintf(stderr,"\nDisplay Mode is Points");
  	handled = true;
  	updateGL();
  }
  else if((e->key()==Qt::Key_D) && (modifiers==Qt::NoButton)){
  	fprintf(stderr,"\nSaving a bunch of Debug Images...");
  	_fusion->saveZimages();
//  	_fusion->saveZimagesFull();
  	fprintf(stderr,"\nDebug Images saved.");
  }



  if (!handled){
    QGLViewer::keyPressEvent(e);
  }
}

typedef struct FusionParameter_{
  FusionMipMapCPU *fusion;
  float imageDepthScale;
  float maxCamDistance;
  bool threadImageReading;
  size_t stopFrame;
  FusionParameter_(FusionMipMapCPU *fusion_p, float imageDepthScale_p, float maxCamDistance_p, bool threadImageReading_p, size_t stopFrame_p)
  :fusion(fusion_p),imageDepthScale(imageDepthScale_p),maxCamDistance(maxCamDistance_p), threadImageReading(threadImageReading_p)
  ,stopFrame(stopFrame_p){}
} FusionParameter;


void imageReadingAndPreprocessingWrapper
(
		std::vector<std::string> depthNames,
		std::vector<std::string> rgbNames,
		unsigned int size,
		unsigned int startFrame,
		unsigned int endFrame,
		volatile cv::Mat **depthImageBuffer,
		volatile std::vector<cv::Mat> **rgbSplitImageBuffer,
		volatile bool *readingActive,
		float maxCamDistance
)
{
	fprintf(stderr,"\nStarted Image Reading Thread");

	for(unsigned int i=startFrame;*readingActive && i<endFrame;i++){
		cv::Mat *depthPointer = new cv::Mat();
		*depthPointer = cv::imread(depthNames[i],-1);
		depthPointer->convertTo(*depthPointer,CV_32FC1,1.0/5000.0);
		depthPointer->setTo(IMAGEINFINITE,*depthPointer == 0.0);
		depthPointer->setTo(IMAGEINFINITE,*depthPointer >= maxCamDistance);

		std::vector<cv::Mat> *rgbPointer = new std::vector<cv::Mat>(3);
		cv::Mat rgbimage = cv::imread(rgbNames[i]);
		cv::split(rgbimage,*rgbPointer);

		depthImageBuffer[i] = depthPointer;
		rgbSplitImageBuffer[i] = rgbPointer;

		fprintf(stderr," I:%i",i);
	}

	fprintf(stderr,"\nFinished Image Reading Thread");
}

void imageReadingWrapper
(
		std::vector<std::string> depthNames,
		std::vector<std::string> rgbNames,
		unsigned int startFrame,
		unsigned int endFrame,
		volatile cv::Mat **depthImageBuffer,
		volatile cv::Mat **rgbImageBuffer,
		volatile bool *readingActive,
		float maxCamDistance,
		volatile unsigned int *deleteUntoHere
)
{
	fprintf(stderr,"\nStarted Image Reading Thread");

	unsigned int deletedFrames = startFrame;

	for(unsigned int i=startFrame;*readingActive && i<endFrame;i++){
		cv::Mat *depthPointer = new cv::Mat();
		*depthPointer = cv::imread(depthNames[i],-1);
		cv::Mat *rgbPointer = new cv::Mat();
		*rgbPointer = cv::imread(rgbNames[i]);

		depthImageBuffer[i] = depthPointer;
		rgbImageBuffer[i] = rgbPointer;

		fprintf(stderr," IC:%i",i);
		while(deletedFrames<*deleteUntoHere){
			delete depthImageBuffer[deletedFrames];
			delete rgbImageBuffer[deletedFrames];
			fprintf(stderr," ID:%i",deletedFrames);
			deletedFrames++;
		}
	}
	while(*readingActive){
		while(deletedFrames<*deleteUntoHere){
			delete depthImageBuffer[deletedFrames];
			delete rgbImageBuffer[deletedFrames];
			fprintf(stderr," ID:%i",deletedFrames);
			deletedFrames++;
		}
	}

	fprintf(stderr,"\nFinished Image Reading Thread");
}

void fusionWrapper
(
	  std::vector<std::vector<std::string> > depthNames,
	  std::vector<std::vector<std::string> > rgbNames,
	  std::vector<std::vector<CameraInfo> > poses,
	  FusionParameter par,
	  volatile long int *_currentFrame,
	  volatile long int *_currentTrajectory,
	  volatile bool *newMesh,
	  volatile bool *fusionActive,
		volatile bool *fusionAlive
)
{
	fprintf(stderr,"\nStarting separate Fusion Thread on Frame %li",*_currentFrame+1);
	FusionMipMapCPU *fusion = par.fusion;
	float imageDepthScale = par.imageDepthScale;
	float maxCamDistance = par.maxCamDistance;
  bool threadImageReading = par.threadImageReading;
  size_t stopFrame = par.stopFrame;

  unsigned int startFrame = *_currentFrame+1;
	unsigned int fusedFrames = startFrame;

	std::vector<CameraInfo> &pLast = poses.back();
	size_t lastFrame = std::min(stopFrame,pLast.size());

	std::vector<std::string> &depthLast = depthNames.back();
	std::vector<std::string> &rgbLast = rgbNames.back();

	if(depthLast.size() != rgbLast.size()){
		fprintf(stderr,"\nERROR: The last Depth and RGB Name Vectors have different Size!");
		return;
	}

	volatile cv::Mat **depthImageBuffer = new volatile cv::Mat*[depthLast.size()];
#ifdef PREPROCESS_IMAGES
	volatile std::vector<cv::Mat> **rgbSplitImageBuffer = new volatile std::vector<cv::Mat> *[rgbLast.size()];
#else
	volatile cv::Mat **rgbImageBuffer = new volatile cv::Mat *[rgbLast.size()];
#endif
	for(unsigned int i=0;i<depthLast.size();i++){
		depthImageBuffer[i] = NULL;
#ifdef PREPROCESS_IMAGES
		rgbSplitImageBuffer[i] = NULL;
#else
		rgbImageBuffer[i] = NULL;
#endif
	}

	bool readingActive = true;

	boost::thread *imageThread = NULL;
  if(threadImageReading){
  	fprintf(stderr,"\nStarting Image Reading in decoupled Thread");
#ifdef PREPROCESS_IMAGES
  	imageThread = new boost::thread(imageReadingAndPreprocessingWrapper,depthLast,rgbLast,depthLast.size(),
  			startFrame,depthLast.size(),depthImageBuffer,rgbSplitImageBuffer,&readingActive,maxCamDistance);
#else
  	imageThread = new boost::thread(imageReadingWrapper,depthLast,rgbLast,
  			startFrame,lastFrame,depthImageBuffer,rgbImageBuffer,&readingActive,maxCamDistance,&fusedFrames);
#endif
  }

	if(poses.size()<=1){

		//TODO: Warum funzt das nicht auch ohne volatile size_t und nur mit volatile pointern?!
		for(volatile size_t currentFrame=startFrame;*fusionAlive && currentFrame<lastFrame;){
			if(*fusionActive){
				if(threadImageReading){
					while(!depthImageBuffer[currentFrame] ||
#ifdef PREPROCESS_IMAGES
							!rgbSplitImageBuffer[currentFrame]
#else
							!rgbImageBuffer[currentFrame]
#endif
							                     )
//						fprintf(stderr," W:%li",currentFrame)
						;
#ifdef PREPROCESS_IMAGES
					DEBUG(fprintf(stderr,"\nAdd Depthmap %li",currentFrame));
					fusion->addMap(*((cv::Mat*)depthImageBuffer[currentFrame]),pLast[currentFrame],
							*((std::vector<cv::Mat>*)rgbSplitImageBuffer[currentFrame]));
#else
					fusion->addMap(*((cv::Mat*)depthImageBuffer[currentFrame]),pLast[currentFrame],
							*((cv::Mat*)rgbImageBuffer[currentFrame]),1.0f/imageDepthScale,maxCamDistance);
#endif
//					fprintf(stderr,"\nDeleting Depth Image %li",currentFrame);
//					delete depthImageBuffer[currentFrame];
#ifdef PREPROCESS_IMAGES
					delete rgbSplitImageBuffer[currentFrame];
#else
//					fprintf(stderr,"\nDeleting RGB Image %li",currentFrame);
//					delete rgbImageBuffer[currentFrame];
#endif

#ifndef DEBUG_NO_MESHES
					*newMesh = fusion->updateMeshes();
#endif

					currentFrame++;
					fusedFrames = currentFrame;
				}
				else{
					eprintf("\nAdd Depthmap %li",currentFrame);
					cv::Mat depthimage = cv::imread(depthLast[currentFrame],-1);
					cv::Mat rgbimage = cv::imread(rgbLast[currentFrame]);
#ifdef PREPROCESS_IMAGES
					depthimage.convertTo(depthimage,CV_32FC1,1/(imageDepthScale));
					depthimage.setTo(IMAGEINFINITE,depthimage == 0.0);
					depthimage.setTo(IMAGEINFINITE,depthimage >= maxCamDistance);
					std::vector<cv::Mat> split(3);
					cv::split(rgbimage,split);
					if(depthimage.empty()) fprintf(stderr,"\nERROR: %s is empty!",
							depthLast[currentFrame].c_str());
					fusion->addMap(depthimage,pLast[currentFrame],split);
#else
					eprintf("\nAdding Integer Depthmap");
					fusion->addMap(depthimage,pLast[currentFrame],rgbimage,1.0f/imageDepthScale,maxCamDistance);
					eprintf("\nInteger Depthmap added.");
#endif

#ifndef DEBUG_NO_MESHES
					*newMesh = fusion->updateMeshes();
#endif

					currentFrame++;
				}
				*_currentFrame = currentFrame;
			}
		}
	}
	else{
		if(*_currentTrajectory<0) *_currentTrajectory = 0;
		for(volatile size_t currentTrajectory=*_currentTrajectory;*fusionAlive && currentTrajectory<poses.size();){
			if(*fusionActive){

				volatile size_t firstImage = currentTrajectory>0 ? poses[currentTrajectory-1].size() : 0;
				volatile size_t lastImage = poses[currentTrajectory].size();

				std::vector<cv::Mat> depthImages;
#ifdef PREPROCESS_IMAGES
				std::vector<std::vector<cv::Mat> > rgbSplitImages;
#else
				std::vector<cv::Mat> rgbImages;
#endif
				volatile size_t currentFrame = firstImage;


				fprintf(stderr,"\nAdding Multiple Images from %li to %li at Trajectory %li",
						firstImage,lastImage,currentTrajectory);

				if(threadImageReading){
					while(!depthImageBuffer[currentFrame] ||
#ifdef PREPROCESS_IMAGES
							!rgbSplitImageBuffer[currentFrame]
#else
							!rgbImageBuffer[currentFrame]
#endif
							                     )
						;

					while(currentFrame<lastImage){
						depthImages.push_back(*((cv::Mat*)depthImageBuffer[currentFrame]));
#ifdef PREPROCESS_IMAGES
						rgbSplitImages.push_back(*((std::vector<cv::Mat>*)rgbSplitImageBuffer[currentFrame]));
#else
						rgbImages.push_back(*((cv::Mat*)rgbImageBuffer[currentFrame]));
#endif
						currentFrame++;
					}
				}
				else{

					while(currentFrame<lastImage){
						depthImages.push_back(cv::Mat());
						cv::Mat &depthimage = depthImages.back();
						depthimage = cv::imread(depthNames[currentTrajectory][currentFrame],-1);
						cv::Mat rgbimage = cv::imread(rgbNames[currentTrajectory][currentFrame]);
#ifdef PREPROCESS_IMAGES
						depthimage.convertTo(depthimage,CV_32FC1,1/(imageDepthScale));
						depthimage.setTo(IMAGEINFINITE,depthimage == 0.0);
						depthimage.setTo(IMAGEINFINITE,depthimage >= maxCamDistance);
						rgbSplitImages.push_back(std::vector<cv::Mat>(3));
						std::vector<cv::Mat> &split = rgbSplitImages.back();
						cv::split(rgbimage,split);
#else
						rgbImages.push_back(rgbimage);
#endif

						currentFrame++;
					}
				}
				currentFrame--;


#ifdef PREPROCESS_IMAGES
				fusion->addMap(depthImages,poses[currentTrajectory],rgbSplitImages,_currentFrame);
#else
//FIXME: Hier muss vektor-addMap mit Integer-Werten rein!
#endif

				depthImages.clear();

#ifdef PREPROCESS_IMAGES
				rgbSplitImages.clear();
#else
			rgbImages.clear();
#endif

				fprintf(stderr,"\nAdded Images up to Frame %li",currentFrame);

				fprintf(stderr,"M");
				*newMesh = fusion->updateMeshes();
				fprintf(stderr,"!");

				fprintf(stderr,"\nTrajectory %li finished",currentTrajectory);
				currentTrajectory++;
				*_currentTrajectory = currentTrajectory;

				currentFrame++;
//				*_currentFrame = currentFrame;
			}
		}
	}

	readingActive= false;

  if(threadImageReading){
  	if(imageThread){
  		imageThread->join();
  		delete imageThread;
  	}
  }

	delete [] depthImageBuffer;
#ifdef PREPROCESS_IMAGES
	delete [] rgbSplitImageBuffer;
#else
	delete [] rgbImageBuffer;
#endif
}

void filterimage(cv::Mat &image)                                                                                                 
{                                                                                                                                

  cv::Mat input = image.clone();                                                                                                 
  for(int x=1;x<image.cols-1;x++){                                                                                               
    for(int y=1;y<image.rows-1;y++){                                                                                             
      if(std::isfinite(input.at<float>(y,x))){                                                                                   
        float sum = 0.0f; float count = 0.0f;                                                                                    
        for(int dx=-1;dx<=1;dx++){                                                                                               
          for(int dy=-1;dy<=1;dy++){                                                                                             
            if(std::isfinite(input.at<float>(y+dy,x+dx)) && fabs(input.at<float>(y,x)-input.at<float>(y+dy,x+dx))<0.1f){         
              sum += input.at<float>(y+dy,x+dx); count += 1.0f;                                                                  
            }                                                                                                                    
          }                                                                                                                      
        }                                                                                                                        
        image.at<float>(y,x) = sum/count;                                                                                        
      }                                                                                                                          
    }                                                                                                                            
  }                                                                                                                              
}                                                                                                                                


void OnlineFusionViewerManipulated::updateSlot()
{
	if(_threadFusion){
		/*if(!_fusionThread){
			_fusionThread = new boost::thread(fusionWrapper,_depthNames,_rgbNames,_poses,
					FusionParameter(_fusion,_imageDepthScale,_maxCamDistance,_threadImageReading,_nextStopFrame),
					&_currentFrame,&_currentTrajectory,&_newMesh,&_fusionActive,&_fusionAlive);
		}

		if(_newMesh){
			_newMesh = false;
			_lastComputedFrame = _currentFrame;
			if(_createMeshList){
				_pointermeshes.resize(_currentFrame+1,NULL);
				if(_pointermeshes[_currentFrame]) delete _pointermeshes[_currentFrame];
				_pointermeshes[_currentFrame] = new PointerMeshDraw(_fusion->getMeshInterleavedMarchingCubes(),_lightingEnabled? 1 : 0);
			}
			else{
				if(!_pointermeshes.size()) _pointermeshes.resize(1,NULL);
				if(_pointermeshes[0]) delete _pointermeshes[0];
				if(!_currentMeshForSave) _currentMeshForSave = new MeshSeparate(3);
				if(!_currentMeshInterleaved) _currentMeshInterleaved = new MeshInterleaved(3);
#ifndef DEBUG_NO_MESHES
#ifndef DEBUG_NO_MESH_VISUALIZATION
				*_currentMeshInterleaved = _fusion->getMeshInterleavedMarchingCubes();
				if(_lightingEnabled) _pointermeshes[0] = new PointerMeshDraw(*_currentMeshInterleaved,_lightingEnabled? 1 : 0);
#endif
#endif
//				fprintf(stderr,"\nCopied Mesh, creating Pointer Mesh");
//				_pointermeshes[0] = new PointerMeshDraw(*_currentMeshForSave,_lightingEnabled? 1 : 0);
//				fprintf(stderr,"\nPointer Mesh created");
			}
		}*/
	}
	else{
      unsigned int keyFrame = 0;
      {
        unique_lock<mutex> lock( keyframeMutex );
        keyFrame = _keyFrame;
      }
      if(keyFrame > _frame)
      {  
			  _frame = keyFrame;
        _currentFrame++;
        cv::Mat depthimage,rgbimage;
        CameraInfo pose;
        {
          unique_lock<mutex> lock( keyframeMutex );
			    depthimage = _depthImg;
			    rgbimage = _colorImg;
          pose = _pose;
        }
        cv::medianBlur (depthimage, depthimage, 5);
			  //depthimage.convertTo(depthimage,CV_32FC1,1/(_imageDepthScale));
        //cv::imwrite("/home/iopenlink/depth_before.png",depthimage);
        //cv::medianBlur (depthimage, depthimage, 5);
        //cv::imwrite("/home/iopenlink/depth_after.png",depthimage);
#ifdef PREPROCESS_IMAGES
			  depthimage.convertTo(depthimage,CV_32FC1,1/(_imageDepthScale));
			  depthimage.setTo(IMAGEINFINITE,depthimage == 0.0);
			  depthimage.setTo(IMAGEINFINITE,depthimage >= _maxCamDistance);
			  std::vector<cv::Mat> split(3);
			  cv::split(rgbimage,split);
			  _fusion->addMap(depthimage,pose,split);
#else
			  _fusion->addMap(depthimage,pose,rgbimage,1.0f/_imageDepthScale,_maxCamDistance);
#endif
#ifndef DEBUG_NO_MESHES 

			  _fusion->updateMeshes();
#endif

			  _lastComputedFrame = _currentFrame;
			  if(_createMeshList){
			    _pointermeshes.resize(_currentFrame+1,NULL);
				  if(_pointermeshes[_currentFrame]) delete _pointermeshes[_currentFrame];
				  _pointermeshes[_currentFrame] = new PointerMeshDraw(_fusion->getMeshInterleavedMarchingCubes(),_lightingEnabled? 1 : 0);
			  }
			  else{
				  if(!_pointermeshes.size()) _pointermeshes.resize(1,NULL);
				  if(_pointermeshes[0]) delete _pointermeshes[0];
				  if(!_currentMeshForSave) _currentMeshForSave = new MeshSeparate(3);
				  if(!_currentMeshInterleaved) _currentMeshInterleaved = new MeshInterleaved(3);
//			  *_currentMeshForSave = _fusion->getMeshSeparateMarchingCubes();
#ifndef DEBUG_NO_MESHES
#ifndef DEBUG_NO_MESH_VISUALIZATION
			    *_currentMeshInterleaved = _fusion->getMeshInterleavedMarchingCubes();
			    if(_lightingEnabled) _pointermeshes[0] = new PointerMeshDraw(*_currentMeshInterleaved,_lightingEnabled? 1 : 0);
#endif
#endif
			    eprintf("\nInterleaved Mesh assigned");
//		    _pointermeshes[0] = new PointerMeshDraw(*_currentMeshForSave,_lightingEnabled? 1 : 0);
			  }
      }
	}
//	fprintf(stderr,"\nUpdate Slot done, updating GL");
	updateGL();
}

} //namespace ORB_SLAM
