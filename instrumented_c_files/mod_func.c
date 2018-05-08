#include <stdio.h>
extern int nrnmpi_myid;
extern int nrn_nobanner_;
extern int _Ca_reg(void),
  _CaDynamics_reg(void),
  _CaDynamics_E2_reg(void),
  _Ca_HVA_reg(void),
  _Ca_LVA_reg(void),
  _Ca_LVAst_reg(void),
  _DetAMPANMDA_reg(void),
  _DetGABAAB_reg(void),
  _Ih_reg(void),
  _Im_reg(void),
  _Im_v2_reg(void),
  _K_P_reg(void),
  _K_Pst_reg(void),
  _K_T_reg(void),
  _K_Tst_reg(void),
  _Kd_reg(void),
  _Kv2like_reg(void),
  _Kv3_1_reg(void),
  _NaTa_reg(void),
  _NaTa_t_reg(void),
  _NaTs_reg(void),
  _NaTs2_t_reg(void),
  _Nap_reg(void),
  _Nap_Et2_reg(void),
  _ProbAMPANMDA_EMS_reg(void),
  _ProbFiltAMPANMDA_EMS_reg(void),
  _ProbFiltGABAAB_EMS_reg(void),
  _ProbGABAAB_EMS_reg(void),
  _SK_reg(void),
  _SK_E2_reg(void),
  _SKv3_1_reg(void),
  _gif_reg(void),
  _gif4_reg(void),
  _halfgap_reg(void),
  _iaf_psc_alpha_reg(void),
  _if3_reg(void),
  _intfirecur_reg(void),
  _opt_ProbAMPANMDA_EMS_reg(void);

void modl_reg() {
    if (!nrn_nobanner_)
        if (nrnmpi_myid < 1) {
            fprintf(stderr, " Additional mechanisms from files\n");

                fprintf(stderr," Ca.mod");
    fprintf(stderr," CaDynamics.mod");
    fprintf(stderr," CaDynamics_E2.mod");
    fprintf(stderr," Ca_HVA.mod");
    fprintf(stderr," Ca_LVA.mod");
    fprintf(stderr," Ca_LVAst.mod");
    fprintf(stderr," DetAMPANMDA.mod");
    fprintf(stderr," DetGABAAB.mod");
    fprintf(stderr," Ih.mod");
    fprintf(stderr," Im.mod");
    fprintf(stderr," Im_v2.mod");
    fprintf(stderr," K_P.mod");
    fprintf(stderr," K_Pst.mod");
    fprintf(stderr," K_T.mod");
    fprintf(stderr," K_Tst.mod");
    fprintf(stderr," Kd.mod");
    fprintf(stderr," Kv2like.mod");
    fprintf(stderr," Kv3_1.mod");
    fprintf(stderr," NaTa.mod");
    fprintf(stderr," NaTa_t.mod");
    fprintf(stderr," NaTs.mod");
    fprintf(stderr," NaTs2_t.mod");
    fprintf(stderr," Nap.mod");
    fprintf(stderr," Nap_Et2.mod");
    fprintf(stderr," ProbAMPANMDA_EMS.mod");
    fprintf(stderr," ProbFiltAMPANMDA_EMS.mod");
    fprintf(stderr," ProbFiltGABAAB_EMS.mod");
    fprintf(stderr," ProbGABAAB_EMS.mod");
    fprintf(stderr," SK.mod");
    fprintf(stderr," SK_E2.mod");
    fprintf(stderr," SKv3_1.mod");
    fprintf(stderr," gif.mod");
    fprintf(stderr," gif4.mod");
    fprintf(stderr," halfgap.mod");
    fprintf(stderr," iaf_psc_alpha.mod");
    fprintf(stderr," if3.mod");
    fprintf(stderr," intfirecur.mod");
    fprintf(stderr," opt_ProbAMPANMDA_EMS.mod"); fprintf(stderr,
                                                                                   "\n\n");
        }

     _Ca_reg();
 _CaDynamics_reg();
 _CaDynamics_E2_reg();
 _Ca_HVA_reg();
 _Ca_LVA_reg();
 _Ca_LVAst_reg();
 _DetAMPANMDA_reg();
 _DetGABAAB_reg();
 _Ih_reg();
 _Im_reg();
 _Im_v2_reg();
 _K_P_reg();
 _K_Pst_reg();
 _K_T_reg();
 _K_Tst_reg();
 _Kd_reg();
 _Kv2like_reg();
 _Kv3_1_reg();
 _NaTa_reg();
 _NaTa_t_reg();
 _NaTs_reg();
 _NaTs2_t_reg();
 _Nap_reg();
 _Nap_Et2_reg();
 _ProbAMPANMDA_EMS_reg();
 _ProbFiltAMPANMDA_EMS_reg();
 _ProbFiltGABAAB_EMS_reg();
 _ProbGABAAB_EMS_reg();
 _SK_reg();
 _SK_E2_reg();
 _SKv3_1_reg();
 _gif_reg();
 _gif4_reg();
 _halfgap_reg();
 _iaf_psc_alpha_reg();
 _if3_reg();
 _intfirecur_reg();
 _opt_ProbAMPANMDA_EMS_reg();
}
