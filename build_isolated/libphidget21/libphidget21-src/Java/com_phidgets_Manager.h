/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_phidgets_Manager */

#ifndef _Included_com_phidgets_Manager
#define _Included_com_phidgets_Manager
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     com_phidgets_Manager
 * Method:    create
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_phidgets_Manager_create
  (JNIEnv *, jobject);

/*
 * Class:     com_phidgets_Manager
 * Method:    getServerAddress
 * Signature: ()Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_com_phidgets_Manager_getServerAddress
  (JNIEnv *, jobject);

/*
 * Class:     com_phidgets_Manager
 * Method:    getServerID
 * Signature: ()Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_com_phidgets_Manager_getServerID
  (JNIEnv *, jobject);

/*
 * Class:     com_phidgets_Manager
 * Method:    getServerPort
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_com_phidgets_Manager_getServerPort
  (JNIEnv *, jobject);

/*
 * Class:     com_phidgets_Manager
 * Method:    isAttached
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_com_phidgets_Manager_isAttached
  (JNIEnv *, jobject);

/*
 * Class:     com_phidgets_Manager
 * Method:    isAttachedToServer
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_com_phidgets_Manager_isAttachedToServer
  (JNIEnv *, jobject);

/*
 * Class:     com_phidgets_Manager
 * Method:    nativeClose
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_phidgets_Manager_nativeClose
  (JNIEnv *, jobject);

/*
 * Class:     com_phidgets_Manager
 * Method:    nativeDelete
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_phidgets_Manager_nativeDelete
  (JNIEnv *, jobject);

/*
 * Class:     com_phidgets_Manager
 * Method:    nativeOpen
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_phidgets_Manager_nativeOpen
  (JNIEnv *, jobject);

/*
 * Class:     com_phidgets_Manager
 * Method:    nativeOpenRemote
 * Signature: (Ljava/lang/String;Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_com_phidgets_Manager_nativeOpenRemote
  (JNIEnv *, jobject, jstring, jstring);

/*
 * Class:     com_phidgets_Manager
 * Method:    nativeOpenRemoteIP
 * Signature: (Ljava/lang/String;ILjava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_com_phidgets_Manager_nativeOpenRemoteIP
  (JNIEnv *, jobject, jstring, jint, jstring);

/*
 * Class:     com_phidgets_Manager
 * Method:    enableManagerAttachEvents
 * Signature: (Z)V
 */
JNIEXPORT void JNICALL Java_com_phidgets_Manager_enableManagerAttachEvents
  (JNIEnv *, jobject, jboolean);

/*
 * Class:     com_phidgets_Manager
 * Method:    enableManagerDetachEvents
 * Signature: (Z)V
 */
JNIEXPORT void JNICALL Java_com_phidgets_Manager_enableManagerDetachEvents
  (JNIEnv *, jobject, jboolean);

/*
 * Class:     com_phidgets_Manager
 * Method:    enableServerConnectEvents
 * Signature: (Z)V
 */
JNIEXPORT void JNICALL Java_com_phidgets_Manager_enableServerConnectEvents
  (JNIEnv *, jobject, jboolean);

/*
 * Class:     com_phidgets_Manager
 * Method:    enableServerDisconnectEvents
 * Signature: (Z)V
 */
JNIEXPORT void JNICALL Java_com_phidgets_Manager_enableServerDisconnectEvents
  (JNIEnv *, jobject, jboolean);

#ifdef __cplusplus
}
#endif
#endif