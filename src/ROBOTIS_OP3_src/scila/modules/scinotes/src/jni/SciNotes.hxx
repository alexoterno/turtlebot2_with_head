/* Generated by GIWS (version 2.0.2) with command:
giws --disable-return-size-array --output-dir src/jni/ --throws-exception-on-error --description-file src/jni/SciNotes.giws.xml
*/
/*

This is generated code.

This software is a computer program whose purpose is to hide the complexity
of accessing Java objects/methods from C++ code.

This software is governed by the CeCILL-B license under French law and
abiding by the rules of distribution of free software.  You can  use,
modify and/ or redistribute the software under the terms of the CeCILL-B
license as circulated by CEA, CNRS and INRIA at the following URL
"http://www.cecill.info".

As a counterpart to the access to the source code and  rights to copy,
modify and redistribute granted by the license, users are provided only
with a limited warranty  and the software's author,  the holder of the
economic rights,  and the successive licensors  have only  limited
liability.

In this respect, the user's attention is drawn to the risks associated
with loading,  using,  modifying and/or developing or reproducing the
software by the user in light of its specific status of free software,
that may mean  that it is complicated to manipulate,  and  that  also
therefore means  that it is reserved for developers  and  experienced
professionals having in-depth computer knowledge. Users are therefore
encouraged to load and test the software's suitability as regards their
requirements in conditions enabling the security of their systems and/or
data to be ensured and,  more generally, to use and operate it in the
same conditions as regards security.

The fact that you are presently reading this means that you have had
knowledge of the CeCILL-B license and that you accept its terms.
*/


#ifndef __ORG_SCILAB_MODULES_SCINOTES_SCINOTES__
#define __ORG_SCILAB_MODULES_SCINOTES_SCINOTES__
#include <iostream>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <jni.h>

#include "GiwsException.hxx"

        #if defined(_MSC_VER) /* Defined anyway with Visual */
            #include <Windows.h>
        #else
            typedef signed char byte;
        #endif


#ifndef GIWSEXPORT
# if defined(_MSC_VER) || defined(__WIN32__) || defined(__CYGWIN__)
#   if defined(STATIC_LINKED)
#     define GIWSEXPORT
#   else
#     define GIWSEXPORT __declspec(dllexport)
#   endif
# else
#   if __GNUC__ >= 4
#     define GIWSEXPORT __attribute__ ((visibility ("default")))
#   else
#     define GIWSEXPORT
#   endif
# endif
#endif

namespace org_scilab_modules_scinotes {
class GIWSEXPORT SciNotes {

private:
JavaVM * jvm;

protected:
jmethodID voidscinotesID; // cache method id
jmethodID voidscinotesjstringjava_lang_StringID; // cache method id
jmethodID voidscinotesjstringjava_lang_Stringjintintjstringjava_lang_StringID; // cache method id
jmethodID voidscinotesjstringjava_lang_StringjobjectArray_java_lang_Stringjava_lang_StringID; // cache method id
jclass stringArrayClass;
jmethodID voidcloseSciNotesFromScilabID; // cache method id



jobject instance;
jclass instanceClass; // cache class

                       
// Caching (if any)


/**
* Get the environment matching to the current thread.
*/
virtual JNIEnv * getCurrentEnv();

public:
// Constructor
/**
* Create a wrapping of the object from a JNIEnv.
* It will call the default constructor
* @param JEnv_ the Java Env
*/
SciNotes(JavaVM * jvm_);

/**
* Create a wrapping of an already existing object from a JNIEnv.
* The object must have already been instantiated
* @param JEnv_ the Java Env
* @param JObj the object
*/
SciNotes(JavaVM * jvm_, jobject JObj);


/** 
* This is a fake constructor to avoid the constructor
* chaining when dealing with extended giws classes 
*/
#ifdef FAKEGIWSDATATYPE
SciNotes(fakeGiwsDataType::fakeGiwsDataType /* unused */) {}
#endif

// Destructor
~SciNotes();

// Generic method
// Synchronization methods
/**
* Enter monitor associated with the object.
* Equivalent of creating a "synchronized(obj)" scope in Java.
*/
void synchronize();

/**
* Exit monitor associated with the object.
* Equivalent of ending a "synchronized(obj)" scope.
*/
void endSynchronize();

// Methods
static void scinotes(JavaVM * jvm_);

static void scinotes(JavaVM * jvm_, char const* fileName);

static void scinotes(JavaVM * jvm_, char const* fileName, int lineNumber, char const* functionName);

static void scinotes(JavaVM * jvm_, char const* fileName, char const* const* option, int optionSize);

static void closeSciNotesFromScilab(JavaVM * jvm_);


                        /**
                        * Get class name to use for static methods
                        * @return class name to use for static methods
                        */
                        
                static const std::string className()
                {
                return "org/scilab/modules/scinotes/SciNotes";
                }
                

                        /**
                        * Get class to use for static methods
                        * @return class to use for static methods
                        */
                        
                static jclass initClass(JNIEnv * curEnv)
                {
                    static jclass cls = 0;

                    if (cls == 0)
                    {
                        jclass _cls = curEnv->FindClass(className().c_str());
                        if (_cls)
                        {
                            cls = static_cast<jclass>(curEnv->NewGlobalRef(_cls));
                        }
                    }

                    return cls;
                 }
                
};


}
#endif
