#include "innermodelManager.h"

InnerModelManager::InnerModelManager(InnerModel *innermodel, InnerModelViewer *imv)
{
  this->innerModel=innermodel;
  this->imv=imv;
}

void InnerModelManager::setPointCloudData(const std::string id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
  QString m = QString("setPointCloudData");
  std::cout<<"InnerModelManager::setPointCloudData: "<<id<<" "<<cloud->size()<<std::endl;
  
  /// Aqui Marco va a mejorar el código :-) felicidad (comprobar que la nube existe)
  IMVPointCloud *pcNode = imv->pointCloudsHash[QString::fromStdString(id)];
  
  int points = cloud->size();
  pcNode->points->resize(points);
  pcNode->colors->resize(points);
  pcNode->setPointSize(4);
  int i = 0;
  for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloud->begin(); it != cloud->end(); it++ )
  {
    if (!pcl_isnan(it->x)&&!pcl_isnan(it->y)&&!pcl_isnan(it->z))
    {
      pcNode->points->operator[](i) = QVecToOSGVec(QVec::vec3(it->x, it->y, it->z));
      //pcNode->colors->operator[](i) = osg::Vec4f(float(it->r)/255, float(it->g)/255, float(it->b)/255, 1.f);
      pcNode->colors->operator[](i) = osg::Vec4f(255.f, 0.f, 0.f, 1.f);
    }
    //std::cout<<i<<": "<<it->x<<" "<<it->y<<" "<<it->z<<std::endl;
    //std::cout<<i<<": "<<uint(it->r)<<" "<<uint(it->g)<<" "<<uint(it->b)<<std::endl;
    i++;
  }  
  pcNode->update();
  imv->update();
  
}

void InnerModelManager::setPose(std::string item,  QVec t,  QVec r,  QVec s)
{
  QString qItem = QString::fromStdString(item);
  QString m="RoboCompInnerModelManager::setPoseFromParent()"; 
  
  InnerModelTransform *aux = dynamic_cast<InnerModelTransform*>(getNode(QString::fromStdString(item),m));
  checkOperationInvalidNode(aux,m + qItem +"can't be use as base because it's not a InnerModelTransform node.");
  
  innerModel->updateTransformValues(qItem, t(0), t(1), t(2), r(0) , r(1) , r(2));
  imv->update();  

//   if (collisiondetection->isChecked())
//   {
//     checkPoseCollision(qItem,m);
//   }

}

void InnerModelManager::setScale(std::string item, float scaleX,float scaleY, float scaleZ)
{
  //QMutexLocker locker (mutex);
  QString qItem = QString::fromStdString(item);
  QString m="RoboCompInnerModelManager::setScale()"; 

  InnerModelMesh *aux = dynamic_cast<InnerModelMesh*>(getNode(QString::fromStdString(item),m));
  checkOperationInvalidNode(aux,m + qItem +"can't be use as base because it's not a InnerModelMesh node.");
  
  aux->setScale(scaleX, scaleY, scaleZ);
  imv->update();  
  
// #ifdef INNERMODELMANAGERDEBUG 
//   try 
//   {
//     checkPoseCollision(qItem,m);
//   }
//   catch (RoboCompInnerModelManager::InnerModelManagerError err)
//   {
//     std::cout<<err.what()<<" "<<err.text<< "\n";
//     std::cout<< "\n";
//     ///come back to t= (t+1) -t
// 
//   //to check => maybe using a tag in the xml (collide="true"  ) to decide if allow collitions or not
//   //  innerModel->updateTransformValues(qItem,p.x, p.y, p.z, p.rx , p.ry, p.rz);
//   //  innerModel->update();
//     throw err;
//   }
// #endif
}

void InnerModelManager::checkOperationInvalidNode(InnerModelNode *node,QString msg)
{
  if (node==NULL)
  {
#ifdef INNERMODELMANAGERDEBUG
    qDebug()<<msg<<node->id<<"is not transform type";
#endif  
//     RoboCompInnerModelManager::InnerModelManagerError err;
//     err.err = RoboCompInnerModelManager::OperationInvalidNode;
    std::ostringstream oss;
    oss <<msg.toStdString()<<" error: Node " << node->id.toStdString()<<" is not a Transform";
//    err.text = oss.str();
//     throw err;
//     
  }
}

InnerModelNode *InnerModelManager::getNode(const QString &id, const QString &msg)
{
  InnerModelNode *node = innerModel->getNode(id);
  if (node==NULL)
  {   
//     RoboCompInnerModelManager::InnerModelManagerError err;
//     err.err = RoboCompInnerModelManager::NonExistingNode;
    std::ostringstream oss;
    oss << msg.toStdString() << " error: Node " << id.toStdString() << " does not exist.";
//     err.text = oss.str();
//     throw err;

  }
  else
  {
    return node;
  }
}
