#include <assert.h>
#include <iostream>
#include <vector>
#include "constraints.h"
#include "cost.h"
#include "localization.h"
#include "obstacle.h"
#include "path.h"

void given_overlapping_paths_returns_crash() {
  Path path;
  path.s = {157.396208735619,157.588817471239,157.781426206858,157.974034942478,158.166643678097,158.359252413716,158.551861149336,158.744469884955,158.937078620574,159.129687356194,159.322296091813,159.514904827433,159.707513563052,159.900122298671,160.092731034291,160.28533976991,160.47794850553,160.670557241149,160.863165976768,161.055774712388,161.248383448007,161.440992183627,161.633600919246,161.826209654865,162.018818390485,162.211427126104,162.404035861723,162.596644597343,162.789253332962,162.981862068582,163.174470804201,163.36707953982,163.55968827544,163.752297011059,163.944905746679,164.137514482298,164.330123217917,164.522731953537,164.715340689156,164.907949424775,165.100558160395,165.293166896014,165.485775631634,165.678384367253,165.870993102872,166.063601838492,166.256210574111,166.448819309731,166.64142804535,166.834036780969,167.026645516589,167.219254252208,167.411862987828,167.604471723447,167.797080459066,167.989689194686,168.182297930305,168.374906665924,168.567515401544,168.760124137163,168.952732872783,169.145341608402,169.337950344021,169.530559079641,169.72316781526,169.91577655088,170.108385286499,170.300994022118,170.493602757738,170.686211493357,170.878820228977,171.071428964596,171.264037700215,171.456646435835,171.649255171454,171.841863907073,172.034472642693,172.227081378312,172.419690113932,172.612298849551,172.80490758517,172.99751632079,173.190125056409,173.382733792029,173.575342527648,173.767951263267,173.960559998887,174.153168734506,174.345777470125,174.538386205745,174.730994941364,174.923603676984,175.116212412603,175.308821148222,175.501429883842,175.694038619461,175.886647355081,176.0792560907,176.271864826319,176.464473561939};

  path.d = {6.00051906953501,6.00050665454798,6.00049411636437,6.0004814801822,6.00046877019239,6.00045600959483,6.0004432206144,6.00043042451696,6.00041764162544,6.00040489133581,6.00039219213315,6.00037956160765,6.00036701647064,6.00035457257065,6.00034224490939,6.00033004765782,6.00031799417215,6.00030609700989,6.00029436794584,6.00028281798818,6.00027145739442,6.00026029568752,6.00024934167181,6.00023860344912,6.00022808843475,6.00021780337351,6.00020775435573,6.00019794683335,6.00018838563586,6.0001790749864,6.00017001851775,6.00016121928838,6.00015267979845,6.00014440200587,6.0001363873423,6.00012863672919,6.00012115059383,6.00011392888532,6.00010697109066,6.00010027625075,6.0000938429764,6.00008766946441,6.00008175351354,6.00007609254057,6.00007068359633,6.00006552338169,6.00006060826366,6.00005593429135,6.00005149721202,6.00004729248712,6.00004331530831,6.00003956061347,6.00003602310277,6.00003269725466,6.0000295773419,6.00002665744762,6.00002393148131,6.00002139319486,6.00001903619861,6.00001685397735,6.00001483990636,6.00001298726742,6.00001128926488,6.00000973904164,6.00000832969522,6.00000705429374,6.00000590589201,6.00000487754748,6.00000396233636,6.00000315336955,6.00000244380875,6.00000182688245,6.00000129590195,6.00000084427741,6.00000046553387,6.00000015332727,5.99999990146048,5.99999970389935,5.9999995547887,5.99999944846838,5.99999937948929,5.99999934262939,5.99999933290975,5.99999934561056,5.99999937628719,5.99999942078616,5.99999947526124,5.99999953618943,5.99999960038697,5.99999966502544,5.99999972764772,5.99999978618404,5.99999983896804,5.99999988475273,5.99999992272657,5.99999995252951,5.99999997426895,5.99999998853584,5.99999999642067,5.9999999995295};

  Localization localization;
  localization.s = 481.696;
  localization.d = 6.0;

  Prediction prediction_2;

  prediction_2.id = 2;
  prediction_2.s = 527.438;
  prediction_2.s_original = 488.247;
  prediction_2.d = 5.699;
  prediction_2.d_original = 5.821;
  
  double cost = evaluate_crash(path, localization, {prediction_2});

  assert(abs(cost - 1.0) < 0.000);
}


int main() {
  given_overlapping_paths_returns_crash();
  std::cout << "done" << std::endl;

  return 0;
}
