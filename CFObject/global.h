
//Anzahl der Links, die ich in dieser Roboterversion verwende
//Die Anzahnl der Gelenke k�nnte auch anders sein, wenn man einen anderen Roboter verwendet
//Die homogenen Transformationen werden in Matlab berechnet. 


#if defined(CFOBJECT_EXPORT_DEF) // inside DLL
#   define CFOBJECT_EXPORT   __declspec(dllexport)
#else // outside DLL
#   define CFOBJECT_EXPORT   __declspec(dllimport)
#endif  // XYZLIBRARY_EXPORT

#define EYE_16 {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}
#define EYE_9 {	1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}

#define NELEM_H (16)
#define DOF_R (10)
#define DOF_E (1)
#define DOF_H (1)
#define ELLIPSE_PARAM_X (0.1582f)
#define ELLIPSE_PARAM_Y (0.2488f)
#define ELLIPSE_PARAM_Z (0.8001f)

#define N_COEFFS 4