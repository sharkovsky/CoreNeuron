/* Created by Language version: 6.2.0 */

#undef DISABLE_OPENACC
#define DISABLE_OPENACC

/* VECTORIZED */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "coreneuron/mech/cfile/scoplib.h"
#undef PI
 
#include "coreneuron/nrnoc/md1redef.h"
#include "coreneuron/nrnconf.h"
#include "coreneuron/nrnoc/multicore.h"

#if defined(_OPENACC) && !defined(DISABLE_OPENACC)
#include "coreneuron/nrniv/nrn_acc_manager.h"

#endif
#include "coreneuron/utils/randoms/nrnran123.h"

#include "coreneuron/nrnoc/md2redef.h"
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#if !defined(DISABLE_HOC_EXP)
#undef exp
#define exp hoc_Exp
#endif
extern double hoc_Exp(double);
#endif
 
#define _thread_present_ /**/ 
 
#if defined(_OPENACC) && !defined(DISABLE_OPENACC)
#include <openacc.h>
#if defined(PG_ACC_BUGS)
#define _PRAGMA_FOR_INIT_ACC_LOOP_ _Pragma("acc parallel loop present(_ni[0:_cntml_actual], _nt_data[0:_nt->_ndata], _p[0:_cntml_padded*_psize], _ppvar[0:_cntml_padded*_ppsize], _vec_v[0:_nt->end], nrn_ion_global_map[0:nrn_ion_global_map_size][0:3], _nt[0:1] _thread_present_) if(_nt->compute_gpu)")
#else
#define _PRAGMA_FOR_INIT_ACC_LOOP_ _Pragma("acc parallel loop present(_ni[0:_cntml_actual], _nt_data[0:_nt->_ndata], _p[0:_cntml_padded*_psize], _ppvar[0:_cntml_padded*_ppsize], _vec_v[0:_nt->end], nrn_ion_global_map[0:nrn_ion_global_map_size], _nt[0:1] _thread_present_) if(_nt->compute_gpu)")
#endif
#define _PRAGMA_FOR_STATE_ACC_LOOP_ _Pragma("acc parallel loop present(_ni[0:_cntml_actual], _nt_data[0:_nt->_ndata], _p[0:_cntml_padded*_psize], _ppvar[0:_cntml_padded*_ppsize], _vec_v[0:_nt->end], _nt[0:1], _ml[0:1] _thread_present_) if(_nt->compute_gpu) async(stream_id)")
#define _PRAGMA_FOR_CUR_ACC_LOOP_ _Pragma("acc parallel loop present(_ni[0:_cntml_actual], _nt_data[0:_nt->_ndata], _p[0:_cntml_padded*_psize], _ppvar[0:_cntml_padded*_ppsize], _vec_v[0:_nt->end], _vec_d[0:_nt->end], _vec_rhs[0:_nt->end], _nt[0:1] _thread_present_) if(_nt->compute_gpu) async(stream_id)")
#define _PRAGMA_FOR_CUR_SYN_ACC_LOOP_ _Pragma("acc parallel loop present(_ni[0:_cntml_actual], _nt_data[0:_nt->_ndata], _p[0:_cntml_padded*_psize], _ppvar[0:_cntml_padded*_ppsize], _vec_v[0:_nt->end], _vec_shadow_rhs[0:_nt->shadow_rhs_cnt], _vec_shadow_d[0:_nt->shadow_rhs_cnt], _vec_d[0:_nt->end], _vec_rhs[0:_nt->end], _nt[0:1]) if(_nt->compute_gpu) async(stream_id)")
#define _PRAGMA_FOR_NETRECV_ACC_LOOP_ _Pragma("acc parallel loop present(_pnt[0:_pnt_length], _nrb[0:1], _nt[0:1], nrn_threads[0:nrn_nthread]) if(_nt->compute_gpu) async(stream_id)")
#define _ACC_GLOBALS_UPDATE_ if (_nt->compute_gpu) {_acc_globals_update();}
#else
#define _PRAGMA_FOR_INIT_ACC_LOOP_ _Pragma("")
#define _PRAGMA_FOR_STATE_ACC_LOOP_ _Pragma("")
#define _PRAGMA_FOR_CUR_ACC_LOOP_ _Pragma("")
#define _PRAGMA_FOR_CUR_SYN_ACC_LOOP_ _Pragma("")
#define _PRAGMA_FOR_NETRECV_ACC_LOOP_ _Pragma("")
#define _ACC_GLOBALS_UPDATE_ ;
#endif
 
#if defined(__clang__)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("clang loop vectorize(enable)")
#elif defined(__ICC) || defined(__INTEL_COMPILER)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("ivdep")
#elif defined(__IBMC__) || defined(__IBMCPP__)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("ibm independent_loop")
#elif defined(__PGI)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("vector")
#elif defined(_CRAYC)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("_CRI ivdep")
#elif defined(__GNUC__) || defined(__GNUG__)
#define _PRAGMA_FOR_VECTOR_LOOP_ _Pragma("GCC ivdep")
#else
#define _PRAGMA_FOR_VECTOR_LOOP_
#endif // _PRAGMA_FOR_VECTOR_LOOP_
 
#if !defined(LAYOUT)
/* 1 means AoS, >1 means AoSoA, <= 0 means SOA */
#define LAYOUT 1
#endif
#if LAYOUT >= 1
#define _STRIDE LAYOUT
#else
#define _STRIDE _cntml_padded + _iml
#endif
 
#define nrn_init _nrn_init__IF3
#define nrn_cur _nrn_cur__IF3
#define _nrn_current _nrn_current__IF3
#define nrn_jacob _nrn_jacob__IF3
#define nrn_state _nrn_state__IF3
#define initmodel initmodel__IF3
#define _net_receive _net_receive__IF3
#define nrn_state_launcher nrn_state_IF3_launcher
#define nrn_cur_launcher nrn_cur_IF3_launcher
#define nrn_jacob_launcher nrn_jacob_IF3_launcher 
#define factors factors_IF3 
#define newstates newstates_IF3 
#define update update_IF3 
 
#define _threadargscomma_ _iml, _cntml_padded, _p, _ppvar, _thread, _nt, v,
#define _threadargsprotocomma_ int _iml, int _cntml_padded, double* _p, Datum* _ppvar, ThreadDatum* _thread, _NrnThread* _nt, double v,
#define _threadargs_ _iml, _cntml_padded, _p, _ppvar, _thread, _nt, v
#define _threadargsproto_ int _iml, int _cntml_padded, double* _p, Datum* _ppvar, ThreadDatum* _thread, _NrnThread* _nt, double v
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define taue _p[0*_STRIDE]
#define taui _p[1*_STRIDE]
#define taum _p[2*_STRIDE]
#define b _p[3*_STRIDE]
#define eps _p[4*_STRIDE]
#define refrac _p[5*_STRIDE]
#define e _p[6*_STRIDE]
#define i _p[7*_STRIDE]
#define m _p[8*_STRIDE]
#define nself _p[9*_STRIDE]
#define nexcite _p[10*_STRIDE]
#define ninhibit _p[11*_STRIDE]
#define gid _p[12*_STRIDE]
#define enew _p[13*_STRIDE]
#define inew _p[14*_STRIDE]
#define mnew _p[15*_STRIDE]
#define t0 _p[16*_STRIDE]
#define ae _p[17*_STRIDE]
#define ai _p[18*_STRIDE]
#define be _p[19*_STRIDE]
#define bi _p[20*_STRIDE]
#define on _p[21*_STRIDE]
#define _v_unused _p[22*_STRIDE]
#define _tsav _p[23*_STRIDE]
 
#ifndef NRN_PRCELLSTATE
#define NRN_PRCELLSTATE 0
#endif
#if NRN_PRCELLSTATE
#define _PRCELLSTATE_V _v_unused = _v;
#define _PRCELLSTATE_G /**/
#else
#define _PRCELLSTATE_V /**/
#define _PRCELLSTATE_G /**/
#endif
#define _nd_area  _nt_data[_ppvar[0*_STRIDE]]
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static ThreadDatum* _extcall_thread;
 /* external NEURON variables */
 
#if 0 /*BBCORE*/
 /* declaration of user functions */
 static double _hoc_E();
 static double _hoc_I();
 static double _hoc_M();
 static double _hoc_firetimebound();
 static double _hoc_factors();
 static double _hoc_newstates();
 static double _hoc_tf();
 static double _hoc_update();
 
#endif /*BBCORE*/
 static int _mechtype;
 extern int nrn_get_mechtype();
extern void hoc_register_prop_size(int, int, int);
extern Memb_func* memb_func;
 static int _pointtype;
 
#if 0 /*BBCORE*/
 static void* _hoc_create_pnt(_ho) Object* _ho; { void* create_point_process();
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt();
 static double _hoc_loc_pnt(_vptr) void* _vptr; {double loc_point_process();
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(_vptr) void* _vptr; {double has_loc_point();
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(_vptr)void* _vptr; {
 double get_loc_point_process(); return (get_loc_point_process(_vptr));
}
 
#endif /*BBCORE*/
 
#if 0 /*BBCORE*/
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 "E", _hoc_E,
 "I", _hoc_I,
 "M", _hoc_M,
 "firetimebound", _hoc_firetimebound,
 "factors", _hoc_factors,
 "newstates", _hoc_newstates,
 "tf", _hoc_tf,
 "update", _hoc_update,
 0, 0
};
 
#endif /*BBCORE*/
#define E E_IF3
#define I I_IF3
#define M M_IF3
#define firetimebound firetimebound_IF3
#define tf tf_IF3
 inline double E( _threadargsproto_ );
 inline double I( _threadargsproto_ );
 inline double M( _threadargsproto_ );
 inline double firetimebound( _threadargsproto_ );
 inline double tf( _threadargsprotocomma_ double );
 /* declare global and static user variables */
 
static void _acc_globals_update() {
 }
 
#if 0 /*BBCORE*/
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "taum", 1e-09, 1e+09,
 "taui", 1e-09, 1e+09,
 "taue", 1e-09, 1e+09,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "taue", "ms",
 "taui", "ms",
 "taum", "ms",
 "refrac", "ms",
 0,0
};
 
#endif /*BBCORE*/
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(double*, Datum*, int);
void nrn_init(_NrnThread*, _Memb_list*, int);
void nrn_state(_NrnThread*, _Memb_list*, int);
 
#if 0 /*BBCORE*/
 static void _hoc_destroy_pnt(_vptr) void* _vptr; {
   destroy_point_process(_vptr);
}
 
#endif /*BBCORE*/
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"IF3",
 "taue",
 "taui",
 "taum",
 "b",
 "eps",
 "refrac",
 0,
 "e",
 "i",
 "m",
 "nself",
 "nexcite",
 "ninhibit",
 "gid",
 0,
 0,
 0};
 
static void nrn_alloc(double* _p, Datum* _ppvar, int _type) {
 
#if 0 /*BBCORE*/
 	/*initialize range parameters*/
 	taue = 3;
 	taui = 10;
 	taum = 30;
 	b = 0;
 	eps = 1e-06;
 	refrac = 5;
 
#endif /* BBCORE */
 
}
 static void _initlists();
 
#define _tqitem &(_nt->_vdata[_ppvar[2*_STRIDE]])
 static void _net_receive(Point_process*, int, double);
 
#define _psize 24
#define _ppsize 3
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*f)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(_threadargsproto_, int));
extern void _cvode_abstol( Symbol**, double*, int);

 void _if3_reg() {
	int _vectorized = 1;
  _initlists();
 _mechtype = nrn_get_mechtype(_mechanism[1]);
 if (_mechtype == -1) return;
 _nrn_layout_reg(_mechtype, LAYOUT);
 
#if 0 /*BBCORE*/
 
#endif /*BBCORE*/
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,(void*)0, (void*)0, (void*)0, nrn_init,
	 hoc_nrnpointerindex,
	 NULL/*_hoc_create_pnt*/, NULL/*_hoc_destroy_pnt*/, /*_member_func,*/
	 1);
  hoc_register_prop_size(_mechtype, _psize, _ppsize);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "netsend");
 add_nrn_artcell(_mechtype, 2);
 add_nrn_has_net_event(_mechtype);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, NULL);
 }
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int factors(_threadargsproto_);
static int newstates(_threadargsprotocomma_ double);
static int update(_threadargsproto_);
 
static int  newstates ( _threadargsprotocomma_ double _ld ) {
   double _lee , _lei , _lem ;
 _lee = exp ( - _ld / taue ) ;
   _lei = exp ( - _ld / taui ) ;
   _lem = exp ( - _ld / taum ) ;
   enew = e * _lee ;
   inew = i * _lei ;
   mnew = b + ( m - b ) * _lem + ae * e * ( taue / ( taum - taue ) ) * ( _lem - _lee ) + ai * i * ( taui / ( taum - taui ) ) * ( _lem - _lei ) ;
    return 0; }
 
#if 0 /*BBCORE*/
 
static double _hoc_newstates(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r = 1.;
 newstates ( _threadargs_, *getarg(1) );
 return(_r);
}
 
#endif /*BBCORE*/
 
double M ( _threadargsproto_ ) {
   double _lM;
 if ( on ) {
     newstates ( _threadargscomma_ t - t0 ) ;
     _lM = mnew ;
     }
   else {
     _lM = 0.0 ;
     }
   
return _lM;
 }
 
#if 0 /*BBCORE*/
 
static double _hoc_M(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  M ( _threadargs_ );
 return(_r);
}
 
#endif /*BBCORE*/
 
double E ( _threadargsproto_ ) {
   double _lE;
 newstates ( _threadargscomma_ t - t0 ) ;
   _lE = enew ;
   
return _lE;
 }
 
#if 0 /*BBCORE*/
 
static double _hoc_E(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  E ( _threadargs_ );
 return(_r);
}
 
#endif /*BBCORE*/
 
double I ( _threadargsproto_ ) {
   double _lI;
 newstates ( _threadargscomma_ t - t0 ) ;
   _lI = inew ;
   
return _lI;
 }
 
#if 0 /*BBCORE*/
 
static double _hoc_I(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  I ( _threadargs_ );
 return(_r);
}
 
#endif /*BBCORE*/
 
static int  update ( _threadargsproto_ ) {
   e = enew ;
   i = inew ;
   m = mnew ;
   t0 = t ;
    return 0; }
 
#if 0 /*BBCORE*/
 
static double _hoc_update(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r = 1.;
 update ( _threadargs_ );
 return(_r);
}
 
#endif /*BBCORE*/
 
static int  factors ( _threadargsproto_ ) {
   double _ltp ;
 if ( taue >= taui ) {
     taui = taue + 0.01 ;
     }
   if ( taum  == taue ) {
     taum = taue + 0.01 ;
     }
   if ( taum  == taui ) {
     taum = taui + 0.01 ;
     }
   _ltp = log ( taue / taum ) / ( ( 1.0 / taum ) - ( 1.0 / taue ) ) ;
   be = 1.0 / ( exp ( - _ltp / taum ) - exp ( - _ltp / taue ) ) ;
   ae = be * ( ( taum / taue ) - 1.0 ) ;
   _ltp = log ( taui / taum ) / ( ( 1.0 / taum ) - ( 1.0 / taui ) ) ;
   bi = 1.0 / ( exp ( - _ltp / taum ) - exp ( - _ltp / taui ) ) ;
   ai = bi * ( ( taum / taui ) - 1.0 ) ;
    return 0; }
 
#if 0 /*BBCORE*/
 
static double _hoc_factors(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r = 1.;
 factors ( _threadargs_ );
 return(_r);
}
 
#endif /*BBCORE*/
 
double tf ( _threadargsprotocomma_ double _lbb ) {
   double _ltf;
 if ( _lbb > 1.0 ) {
     _ltf = taum * log ( ( _lbb - m ) / ( _lbb - 1.0 ) ) ;
     }
   else {
     _ltf = 1e9 ;
     }
   
return _ltf;
 }
 
#if 0 /*BBCORE*/
 
static double _hoc_tf(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  tf ( _threadargs_, *getarg(1) );
 return(_r);
}
 
#endif /*BBCORE*/
 
double firetimebound ( _threadargsproto_ ) {
   double _lfiretimebound;
 double _lh , _ltemp ;
 _lh = ae * e + ai * i ;
   if ( b > 1.0 ) {
     if ( _lh > 0.0 ) {
       _lfiretimebound = tf ( _threadargscomma_ _lh + b ) ;
       }
     else {
       _ltemp = tf ( _threadargscomma_ b ) ;
       _ltemp = tf ( _threadargscomma_ b + _lh * exp ( - _ltemp / taui ) ) ;
       _lfiretimebound = tf ( _threadargscomma_ b + _lh * exp ( - _ltemp / taui ) ) ;
       }
     }
   else {
     if ( _lh + b > 1.0 ) {
       _lfiretimebound = tf ( _threadargscomma_ _lh + b ) ;
       }
     else {
       _lfiretimebound = 1e9 ;
       }
     }
   
return _lfiretimebound;
 }
 
#if 0 /*BBCORE*/
 
static double _hoc_firetimebound(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; ThreadDatum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  firetimebound ( _threadargs_ );
 return(_r);
}
 
#endif /*BBCORE*/
 
static void _net_receive (Point_process* _pnt, int _weight_index, double _lflag) 
{  double* _p; Datum* _ppvar; ThreadDatum* _thread; double v;
   _Memb_list* _ml; int _cntml_padded, _cntml_actual; int _iml; double* _args;
 
   _NrnThread* _nt;
   int _tid = _pnt->_tid; 
   _nt = nrn_threads + _tid;
   _thread = (ThreadDatum*)0; 
   double *_weights = _nt->_weights;
   _args = _weights + _weight_index;
   _ml = _nt->_ml_list[_pnt->_type];
   _cntml_actual = _ml->_nodecount;
   _cntml_padded = _ml->_nodecount_padded;
   _iml = _pnt->_i_instance;
#if LAYOUT == 1 /*AoS*/
   _p = _ml->_data + _iml*_psize; _ppvar = _ml->_pdata + _iml*_ppsize;
#endif
#if LAYOUT == 0 /*SoA*/
   _p = _ml->_data; _ppvar = _ml->_pdata;
#endif
#if LAYOUT > 1 /*AoSoA*/
#error AoSoA not implemented.
#endif
  #if !defined(_OPENACC) 
 assert(_tsav <= t); 
 #endif 
 _tsav = t; 
#if !NET_RECEIVE_BUFFERING
  if (_lflag == 1. ) {*(_tqitem) = 0;}
#endif
 {
   newstates ( _threadargscomma_ t - t0 ) ;
   update ( _threadargs_ ) ;
   if ( _lflag  == 1.0 ) {
     nself = nself + 1.0 ;
     if ( m > 1.0 - eps ) {
       if ( m > 1.0 + eps ) {
         printf ( "m>1 error in IF3 mechanism--m = %g\n" , m ) ;
         }
       net_event ( _pnt, t ) ;
       on = 0.0 ;
       m = 0.0 ;
       artcell_net_send ( _tqitem, _weight_index, _pnt, t +  refrac , 2.0 ) ;
       }
     else {
       artcell_net_send ( _tqitem, _weight_index, _pnt, t +  firetimebound ( _threadargs_ ) , 1.0 ) ;
       }
     }
   else if ( _lflag  == 2.0 ) {
     on = 1.0 ;
     m = 0.0 ;
     artcell_net_send ( _tqitem, _weight_index, _pnt, t +  firetimebound ( _threadargs_ ) , 1.0 ) ;
     }
   else {
     if ( _args[0] > 0.0 ) {
       nexcite = nexcite + 1.0 ;
       e = e + _args[0] ;
       }
     else {
       ninhibit = ninhibit + 1.0 ;
       i = i + _args[0] ;
       }
     if ( on ) {
       artcell_net_move ( _tqitem, _pnt, firetimebound ( _threadargs_ ) + t ) ;
       }
     }
   } 
#if NET_RECEIVE_BUFFERING
#undef t
#define t _nt->_t
#endif
 }

static void initmodel(_threadargsproto_) {
  int _i; double _save;{
 {
   factors ( _threadargs_ ) ;
   e = 0.0 ;
   i = 0.0 ;
   m = 0.0 ;
   t0 = t ;
   on = 1.0 ;
   artcell_net_send ( _tqitem, -1, _nt->_vdata[_ppvar[1*_STRIDE]], t +  firetimebound ( _threadargs_ ) , 1.0 ) ;
   nself = 0.0 ;
   nexcite = 0.0 ;
   ninhibit = 0.0 ;
   }

}
}

void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; ThreadDatum* _thread;
double _v, v; int* _ni; int _iml, _cntml_padded, _cntml_actual;
    _ni = _ml->_nodeindices;
_cntml_actual = _ml->_nodecount;
_cntml_padded = _ml->_nodecount_padded;
_thread = _ml->_thread;

#if defined(PG_ACC_BUGS)
#if defined(celsius)
#undef celsius;
_celsius_ = celsius;
#pragma acc update device (_celsius_) if(_nt->compute_gpu)
#define celsius _celsius_
#endif
#endif
_ACC_GLOBALS_UPDATE_
double * _nt_data = _nt->_data;
double * _vec_v = _nt->_actual_v;
int stream_id = _nt->stream_id;
#if LAYOUT == 1 /*AoS*/
for (_iml = 0; _iml < _cntml_actual; ++_iml) {
 _p = _ml->_data + _iml*_psize; _ppvar = _ml->_pdata + _iml*_ppsize;
#elif LAYOUT == 0 /*SoA*/
 _p = _ml->_data; _ppvar = _ml->_pdata;
_PRAGMA_FOR_INIT_ACC_LOOP_
for (_iml = 0; _iml < _cntml_actual; ++_iml) {
#else /* LAYOUT > 1 */ /*AoSoA*/
#error AoSoA not implemented.
for (;;) { /* help clang-format properly indent */
#endif
 _tsav = -1e20;
 initmodel(_threadargs_);
}
}

static double _nrn_current(_threadargsproto_, double _v){double _current=0.;v=_v;{
} return _current;
}

#if defined(ENABLE_CUDA_INTERFACE) && defined(_OPENACC)
  void nrn_state_launcher(_NrnThread*, _Memb_list*, int, int);
  void nrn_jacob_launcher(_NrnThread*, _Memb_list*, int, int);
  void nrn_cur_launcher(_NrnThread*, _Memb_list*, int, int);
#endif


void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; ThreadDatum* _thread;
double v, _v = 0.0; int* _ni; int _iml, _cntml_padded, _cntml_actual;
    _ni = _ml->_nodeindices;
_cntml_actual = _ml->_nodecount;
_cntml_padded = _ml->_nodecount_padded;
_thread = _ml->_thread;

#if defined(ENABLE_CUDA_INTERFACE) && defined(_OPENACC) && !defined(DISABLE_OPENACC)
  _NrnThread* d_nt = acc_deviceptr(_nt);
  _Memb_list* d_ml = acc_deviceptr(_ml);
  nrn_state_launcher(d_nt, d_ml, _type, _cntml_actual);
  return;
#endif

double * _nt_data = _nt->_data;
double * _vec_v = _nt->_actual_v;
int stream_id = _nt->stream_id;
#if LAYOUT == 1 /*AoS*/
for (_iml = 0; _iml < _cntml_actual; ++_iml) {
 _p = _ml->_data + _iml*_psize; _ppvar = _ml->_pdata + _iml*_ppsize;
#elif LAYOUT == 0 /*SoA*/
 _p = _ml->_data; _ppvar = _ml->_pdata;
/* insert compiler dependent ivdep like pragma */
_PRAGMA_FOR_VECTOR_LOOP_
_PRAGMA_FOR_STATE_ACC_LOOP_
for (_iml = 0; _iml < _cntml_actual; ++_iml) {
#else /* LAYOUT > 1 */ /*AoSoA*/
#error AoSoA not implemented.
for (;;) { /* help clang-format properly indent */
#endif
 v=_v;
{
}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
 int _cntml_actual=1;
 int _cntml_padded=1;
 int _iml=0;
  if (!_first) return;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
 
