/**
 * File: FORB.cpp
 * Date: June 2012
 * Author: Dorian Galvez-Lopez
 * Description: functions for ORB descriptors
 * License: see the LICENSE.txt file
 *
 * Distance function has been modified 
 *
 */

 
#include <vector>
#include <string>
#include <sstream>
#include <stdint-gcc.h>

#include "FORB.h"

using namespace std;

namespace DBoW2 {

// --------------------------------------------------------------------------

const int FORB::L=128;

void FORB::meanValue(const std::vector<FORB::pDescriptor> &descriptors, 
  FORB::TDescriptor &mean)
{
  if(descriptors.empty())
  {
    mean.release();
    return;
  }
  else if(descriptors.size() == 1)
  {
    mean = descriptors[0]->clone();
  }
  else
  {
    mean.resize(FORB::L, 0);

    vector<FORB::pDescriptor>::const_iterator it;
    for (it = descriptors.begin(); it!=descriptors.end(); ++it)
    {
      const FORB::TDescriptor &desc = **it;;
      for (int i = 0; i < FORB::L; ++i)
      {
        mean.at<float>(i) += desc.at<float>(i);
      }
    }
  }
  mean = mean / static_cast<float>(descriptors.size());
}

// --------------------------------------------------------------------------
  
float FORB::distance(const FORB::TDescriptor &a,
  const FORB::TDescriptor &b)
{

  return cv::norm(a, b);
  // Bit set count operation from
  // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel

  const int *pa = a.ptr<int32_t>();
  const int *pb = b.ptr<int32_t>();

  int dist=0;

  for(int i=0; i<8; i++, pa++, pb++)
  {
      unsigned  int v = *pa ^ *pb;
      v = v - ((v >> 1) & 0x55555555);
      v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
      dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}

// --------------------------------------------------------------------------
  
std::string FORB::toString(const FORB::TDescriptor &a)
{
  stringstream ss;
  const float *p = a.ptr<float>();
  
  for(int i = 0; i < a.cols; ++i, ++p)
  {
    ss << (float )*p << " ";
  }
  
  return ss.str();
}

// --------------------------------------------------------------------------
  
void FORB::fromString(FORB::TDescriptor &a, const std::string &s)
{
  a.create(1, FORB::L, CV_32F);
  float *p = a.ptr<float>();
  
  stringstream ss(s);
  for(int i = 0; i < FORB::L; ++i, ++p)
  {
    float n;
    ss >> n;
    
    if(!ss.fail()) 
      *p = (float) n;
  }
  
}

// --------------------------------------------------------------------------

void FORB::toMat32F(const std::vector<TDescriptor> &descriptors, 
  cv::Mat &mat)
{
  if(descriptors.empty())
  {
    mat.release();
    return;
  }
  
  const size_t N = descriptors.size();
  
  mat.create(N, FORB::L, CV_32F);
  float *p = mat.ptr<float>();
  
  for(size_t i = 0; i < N; ++i)
  {
    const int C = descriptors[i].cols;
    const float* desc = descriptors[i].ptr<float>();
    
    for(int j = 0; j < C; ++j, p += 1)
    {
      *p = desc.at<float>(j);
      // p[0] = (desc[j] & (1 << 7) ? 1 : 0);
      // p[1] = (desc[j] & (1 << 6) ? 1 : 0);
      // p[2] = (desc[j] & (1 << 5) ? 1 : 0);
      // p[3] = (desc[j] & (1 << 4) ? 1 : 0);
      // p[4] = (desc[j] & (1 << 3) ? 1 : 0);
      // p[5] = (desc[j] & (1 << 2) ? 1 : 0);
      // p[6] = (desc[j] & (1 << 1) ? 1 : 0);
      // p[7] = desc[j] & (1);
    }
  } 
}

// --------------------------------------------------------------------------

void FORB::toMat8U(const std::vector<TDescriptor> &descriptors, 
  cv::Mat &mat)
{
  mat.create(descriptors.size(), 32, CV_8U);
  
  unsigned char *p = mat.ptr<unsigned char>();
  
  for(size_t i = 0; i < descriptors.size(); ++i, p += 32)
  {
    const unsigned char *d = descriptors[i].ptr<unsigned char>();
    std::copy(d, d+32, p);
  }
  
}

// --------------------------------------------------------------------------

} // namespace DBoW2


