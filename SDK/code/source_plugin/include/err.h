/** @file */
#ifndef _ERRORS_H
#define _ERRORS_H


/// Check if result is OK.
#define IS_OK(val)            ((val) >= 0L)

/// Check if result is failed.
#define IS_FAILED(val)        ((val) < 0L)

/// Return status ERR_NOERROR from the current function.
#define RETURN_NOERROR      return ERR_NONE

/// Return from the current function using a specific error code.
#define RETURN_ERROR(code)  return (code)

/// Return from current function is expression is failed. 
#define RETURN_IF_FAILED(s) {int32_t __result = (s); if (IS_FAILED(__result)) {RETURN_ERROR(__result);}}

/// Return from current function is pointer is NULL. The error code that is returned is ERR_POINTER.
#define RETURN_IF_POINTER_NULL(_ptr)    if ((void *) (_ptr) == NULL) {return -3 ;}

/// Return from current function is pointer is NULL. The error code that is returned is ERR_POINTER.
#define RETURN_IF_POINTER_NULL_WITH_ERROR_CODE(_ptr)    if ((void *) (_ptr) == NULL) {RETURN_ERROR(ERR_POINTER);}

/// This macro asserts if a pointer is NULL.
/// This assertion is only present in Debug builds and it does nothing on Release builds.
#ifdef _DEBUG
    #define ASSERT_IF_POINTER_NULL(_ptr)    __assert((tVoid*) ( _ptr ) != NULL);
#else
    #define ASSERT_IF_POINTER_NULL(_ptr)
#endif

    /// @addtogroup o3d3xx_camera_class O3D3xx Camera Class
/// @{
/// @addtogroup error_codes Status Codes
/// @{
enum ERROR_TYPE
{

    ERR_NONE                        =      0, /**< No error */
    ERR_GENERIC                     =     -1, /**< Generic error i.e. where specific error code is not needed*/
    ERR_UNKNOWN                     =     -2, /**< The function does not know what kind of error occurred*/
    ERR_POINTER                     =     -3, /**< Error with a pointer i.e. access violation etc.*/
    ERR_INVALID_ARG                 =     -4, /**< Invalid argument passed to function*/
    ERR_INVALID_FUNCTION            =     -5, /**< Invalid function called, especially in case of DLLs*/
    ERR_INVALID_ADDRESS             =     -6, /**< Invalid address of memory being used*/
    ERR_INVALID_HANDLE              =     -7, /**< Invalid handle of object being passed to function*/
    ERR_INVALID_FLAGS               =     -8, /**< Invalid or incompatible flags passed to function*/
    ERR_INVALID_INDEX               =     -9, /**< Invalid index of array, list etc being used*/
    ERR_INVALID_FILE                =    -10, /**< Invalid file passed as input*/
    ERR_MEMORY                      =    -11, /**< Generic memory error i.e. unable to read or write etc.*/
    ERR_NOT_IMPL                    =    -12, /**< The function denoted is not implemented as of this version of the dll*/
    ERR_DEVICE_IO                   =    -13, /**< Error when accessing data from device*/
    ERR_DEVICE_NOT_READY            =    -14, /**< Device is not ready*/
    ERR_DEVICE_IN_USE               =    -15, /**< Device is already being used by some other process*/
    ERR_DEVICE_NOT_CONNECTED        =    -16, /**< Device is not connected*/
    ERR_END_OF_FILE                 =    -17, /**< End of file reached when trying to read data*/
    ERR_INVALID_STATE               =    -18, /**< Code execution is in an invalid state*/

        /// @}
    /// @}
};
/// @cond NOT_TO_BE_EXPOSED
typedef enum ERROR_TYPE error_t;
///@endcond


#endif //_ERRORS_H