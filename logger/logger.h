#ifndef _LOGGER_H_
#define _LOGGER_H_


 
#ifdef _MSC_VER
#ifndef GLOG_NO_ABBREVIATED_SEVERITIES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#endif // !GLOG_NO_ABBREVIATED_SEVERITIES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#ifndef GOOGLE_GLOG_DLL_DECL
#define GOOGLE_GLOG_DLL_DECL
#endif // !GOOGLE_GLOG_DLL_DECL
#define GOOGLE_GLOG_DLL_DECL
#pragma comment(lib,"glog.lib")
#endif
#include "glog/logging.h"
#endif // !_LOGGER_H_
