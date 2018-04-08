#ifndef XML_READER_H
#define XML_READER_H

#define TIXML_USE_STL
#include <tinyxml/tinyxml.h>

#include <planning/MultiRobot2DCSpace.h>
#include <planning/AnyMotionPlanner.h>

/** Low-level element parsers **/
bool XmlParse(TiXmlElement* e,Vector2& p);
bool XmlParse(TiXmlElement* e,AABB2D& aabb);
bool XmlParse(TiXmlElement* e,Box2D& box);
bool XmlParse(TiXmlElement* e,Circle2D& circle);
bool XmlParse(TiXmlElement* e,Triangle2D& tri);
bool XmlParse(TiXmlElement* e,Vector3& p);
bool XmlParse(TiXmlElement* e,Geometric2DCollection& geom);
bool XmlParse(TiXmlElement* e,Geometric2DCSpace& space);
bool XmlParse(TiXmlElement* e,RigidRobot2DCSpace& space);
bool XmlParse(TiXmlElement* e,TranslatingRobot2DCSpace& space);
bool XmlParse(TiXmlElement* e,MultiRobot2DCSpace& space);
bool XmlParse(TiXmlElement* e,MotionPlannerFactory& factory);

/** For reading elements as a child of e */
bool XmlRead(TiXmlElement* e,Geometric2DCollection& geom);
bool XmlRead(TiXmlElement* e,Geometric2DCSpace& space);
bool XmlRead(TiXmlElement* e,RigidRobot2DCSpace& space);
bool XmlRead(TiXmlElement* e,TranslatingRobot2DCSpace& space);
bool XmlRead(TiXmlElement* e,MultiRobot2DCSpace& space);


/** @brief A wrapper class, in case we ever change the backend xml parser */
struct XmlDocument
{
  inline bool Load(const char* fn);

  inline TiXmlElement* RootElement();

  template <class T>
  inline bool Read(const char* elementName,T& data);

  TiXmlDocument doc;
};

bool XmlDocument::Load(const char* fn)
{
  return doc.LoadFile(fn);
}

TiXmlElement* XmlDocument::RootElement()
{
  return doc.RootElement();
}

/// A helper template function that just parses a single element from e
template <class T>
bool XmlRead(TiXmlElement* e,const char* elementName,T& data)
{
  TiXmlElement* echild = e->FirstChildElement(elementName);
  if(!echild) {
    printf("Error, element %s, row %d contains no %s children\n",e->Value(),e->Row(),elementName);
    return false;
  }
  if(echild->NextSiblingElement(elementName)) {
    printf("Warning, element %s, row %d contains more than 1 %s children\n",e->Value(),e->Row(),elementName);
  }
  return XmlParse(echild,data);
}

/// A helper template function that parses the index'th element from e
template <class T>
bool XmlRead(TiXmlElement* e,const char* elementName,int index,T& data)
{
  TiXmlElement* echild = e->FirstChildElement(elementName);
  while(echild) {
    if(index == 0) 
      return XmlParse(echild,data);
    echild = echild->NextSiblingElement(elementName);
    index--;
  }
  
  printf("Error, element %s, row %d does not contain %d children of type %s\n",e->Value(),e->Row(),index,elementName);
  return false;
}

template <class T>
bool XmlDocument::Read(const char* elementName,T& data)
{
  return XmlRead(RootElement(),elementName,data);
}



#endif
