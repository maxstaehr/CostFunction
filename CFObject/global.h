
//Anzahl der Links, die ich in dieser Roboterversion verwende
//Die Anzahnl der Gelenke könnte auch anders sein, wenn man einen anderen Roboter verwendet
//Die homogenen Transformationen werden in Matlab berechnet. 
#define N_ELEMENT_T 10

#if defined(CFOBJECT_EXPORT) // inside DLL
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