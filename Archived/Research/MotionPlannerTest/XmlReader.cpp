#include "XmlReader.h"

bool XmlParse(TiXmlElement* e,MotionPlannerFactory& factory)
{
  return factory.Load(e);
}

bool XmlParse(TiXmlElement* e,Vector2& p)
{
  if(e->QueryValueAttribute("value",&p)!=TIXML_SUCCESS) {
    if(e->QueryValueAttribute("x",&p.x)!=TIXML_SUCCESS) {
      printf("Could not read Vector2 attribute x\n");
      return false;
    }
    if(e->QueryValueAttribute("y",&p.y)!=TIXML_SUCCESS) {
      printf("Could not read Vector2 attribute y\n");
      return false;
    }
  }
  return true;
}

bool XmlParse(TiXmlElement* e,Vector3& p)
{
  if(e->QueryValueAttribute("value",&p)!=TIXML_SUCCESS) {
    if(e->QueryValueAttribute("x",&p.x)!=TIXML_SUCCESS) {
      printf("Could not read Vector3 attribute x\n");
      return false;
    }
    if(e->QueryValueAttribute("y",&p.y)!=TIXML_SUCCESS) {
      printf("Could not read Vector3 attribute y\n");
      return false;
    }
    if(e->QueryValueAttribute("z",&p.z)!=TIXML_SUCCESS) {
      printf("Could not read Vector3 attribute z\n");
      return false;
    }
  }
  return true;
}


bool XmlParse(TiXmlElement* e,AABB2D& aabb)
{
  if(e->QueryValueAttribute("bmin",&aabb.bmin)!=TIXML_SUCCESS) {
    printf("Could not read AABB2D attribute bmin\n");
    return false;
  }
  if(e->QueryValueAttribute("bmax",&aabb.bmax)!=TIXML_SUCCESS) {
    printf("Could not read AABB2D attribute bmax\n");
    return false;
  }
  return true;
}


bool XmlParse(TiXmlElement* e,Triangle2D& t)
{
  if(e->QueryValueAttribute("a",&t.a)!=TIXML_SUCCESS) {
    printf("Could not read Triangle2D attribute a\n");
    return false;
  }
  if(e->QueryValueAttribute("b",&t.b)!=TIXML_SUCCESS) {
    printf("Could not read Triangle2D attribute b\n");
    return false;
  }
  if(e->QueryValueAttribute("c",&t.c)!=TIXML_SUCCESS) {
    printf("Could not read Triangle2D attribute c\n");
    return false;
  }
  return true;
}

bool XmlParse(TiXmlElement* e,Circle2D& c)
{
  if(e->QueryValueAttribute("center",&c.center)!=TIXML_SUCCESS) {
    printf("Could not read Circle2D attribute center\n");
    return false;
  }
  if(e->QueryValueAttribute("radius",&c.radius)!=TIXML_SUCCESS) {
    printf("Could not read Circle2D attribute radius\n");
    return false;
  }
  return true;
}

bool XmlParse(TiXmlElement* e,Box2D& b)
{
  if(e->QueryValueAttribute("origin",&b.origin)!=TIXML_SUCCESS) {
    printf("Could not read Box2D attribute origin\n");
    return false;
  }
  Real angle;
  if(e->QueryValueAttribute("angle",&angle)!=TIXML_SUCCESS) {
    printf("Could not read Box2D attribute angle\n");
    return false;
  }
  b.xbasis.set(Cos(angle),Sin(angle));
  b.ybasis.set(-Sin(angle),Cos(angle));
  if(e->QueryValueAttribute("dims",&b.dims)!=TIXML_SUCCESS) {
    printf("Could not read Box2D attribute dims\n");
    return false;
  }
  return true;
}

bool XmlParse(TiXmlElement* e,Geometric2DCollection& geom)
{
  geom.Clear();
  TiXmlElement* eattr = e->FirstChildElement("aabb");
  while(eattr) {
    geom.aabbs.resize(geom.aabbs.size()+1);
    if(!XmlParse(eattr,geom.aabbs.back())) return false;
    eattr = eattr->NextSiblingElement("aabb");
  }
  eattr = e->FirstChildElement("box");
  while(eattr) {
    geom.boxes.resize(geom.boxes.size()+1);
    if(!XmlParse(eattr,geom.boxes.back())) return false;
    eattr = eattr->NextSiblingElement("box");
  }
  eattr = e->FirstChildElement("triangle");
  while(eattr) {
    geom.triangles.resize(geom.triangles.size()+1);
    if(!XmlParse(eattr,geom.triangles.back())) return false;
    eattr = eattr->NextSiblingElement("triangle");
  }
  eattr = e->FirstChildElement("circle");
  while(eattr) {
    geom.circles.resize(geom.circles.size()+1);
    if(!XmlParse(eattr,geom.circles.back())) return false;
    eattr = eattr->NextSiblingElement("circle");
  }
  return true;
}

bool XmlRead(TiXmlElement* e,Geometric2DCollection& geom)
{
  return XmlRead(e,"geometry2d",geom);
}




bool XmlParse(TiXmlElement* e,Geometric2DCSpace& cspace)
{
  Real visibilityEpsilon = cspace.visibilityEpsilon;
  if(e->QueryValueAttribute("visibilityEpsilon",&cspace.visibilityEpsilon)!=TIXML_SUCCESS) {
    cspace.visibilityEpsilon = visibilityEpsilon;
  }
  TiXmlElement* edomain = e->FirstChildElement("domain");  
  if(edomain) {
    if(!XmlParse(edomain,cspace.domain)) return false;
  }
  TiXmlElement* eobs = e->FirstChildElement("obstacles");  
  if(eobs) {
    if(!XmlRead(eobs,(Geometric2DCollection&)cspace)) return false;
  }
  cspace.InitConstraints();
  return true;
}

bool XmlParse(TiXmlElement* e,RigidRobot2DCSpace& cspace)
{
  Real visibilityEpsilon;
  if(e->QueryValueAttribute("visibilityEpsilon",&visibilityEpsilon)==TIXML_SUCCESS) {
    cspace.visibilityEpsilon = visibilityEpsilon;
  }
  Real angleWeight;
  if(e->QueryValueAttribute("angleWeight",&angleWeight)==TIXML_SUCCESS) {
    cspace.SetAngleWeight(angleWeight);
  }
  TiXmlElement* edomain = e->FirstChildElement("domain");  
  if(edomain) {
    if(!XmlParse(edomain,cspace.domain)) return false;
  }
  TiXmlElement* eobs = e->FirstChildElement("obstacles");  
  if(eobs) {
    if(!XmlRead(eobs,cspace.obstacles)) return false;
  }
  TiXmlElement* erobot = e->FirstChildElement("robot");  
  if(erobot) {
    if(!XmlRead(erobot,cspace.robot)) return false;
  }
  cspace.InitConstraints();
  return true;
}

bool XmlParse(TiXmlElement* e,TranslatingRobot2DCSpace& cspace)
{
  Real visibilityEpsilon;
  if(e->QueryValueAttribute("visibilityEpsilon",&visibilityEpsilon)==TIXML_SUCCESS) {
    cspace.visibilityEpsilon = visibilityEpsilon;
  }
  TiXmlElement* edomain = e->FirstChildElement("domain");  
  if(edomain) {
    if(!XmlParse(edomain,cspace.domain)) return false;
  }
  TiXmlElement* eobs = e->FirstChildElement("obstacles");  
  if(eobs) {
    if(!XmlRead(eobs,cspace.obstacles)) return false;
  }
  TiXmlElement* erobot = e->FirstChildElement("robot");  
  if(erobot) {
    if(!XmlRead(erobot,cspace.robot)) return false;
  }
  cspace.InitConstraints();
  return true;
}

bool XmlParse(TiXmlElement* e,MultiRobot2DCSpace& cspace)
{
  bool allowRotation;
  if(e->QueryValueAttribute("allowRotation",&allowRotation)==TIXML_SUCCESS) {
    cspace.allowRotation = allowRotation;
  }
  Real visibilityEpsilon;
  if(e->QueryValueAttribute("visibilityEpsilon",&visibilityEpsilon)==TIXML_SUCCESS) {
    cspace.visibilityEpsilon = visibilityEpsilon;
  }
  TiXmlElement* edomain = e->FirstChildElement("domain");  
  if(edomain) {
    if(!XmlParse(edomain,cspace.domain)) {
      printf("Error reading domain on row %d\n",edomain->Row());
      return false;
    }
  }
  TiXmlElement* eobs = e->FirstChildElement("obstacles");  
  if(eobs) {
    if(!XmlRead(eobs,cspace.obstacles)) {
      printf("Error reading obstacles on row %d\n",eobs->Row());
      return false;
    }
  }
  cspace.robots.resize(0);
  TiXmlElement* erobot = e->FirstChildElement("robot");  
  while(erobot) {
    cspace.robots.resize(cspace.robots.size()+1);
    if(!XmlRead(erobot,cspace.robots.back())) {
      printf("Error reading robot on row %d\n",erobot->Row());
      return false; 
    }
    erobot = erobot->NextSiblingElement("robot");
  }
  return true;
}



bool XmlRead(TiXmlElement* e,Geometric2DCSpace& cspace)
{
  return XmlRead(e,"point2d_cspace",cspace);
}

bool XmlRead(TiXmlElement* e,RigidRobot2DCSpace& cspace)
{
  return XmlRead(e,"rigid2d_cspace",cspace);
}

bool XmlRead(TiXmlElement* e,TranslatingRobot2DCSpace& cspace)
{
  return XmlRead(e,"translating2d_cspace",cspace);
}

bool XmlRead(TiXmlElement* e,MultiRobot2DCSpace& cspace)
{
  return XmlRead(e,"multirobot2d_cspace",cspace);
}
