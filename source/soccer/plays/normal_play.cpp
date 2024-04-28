#include "../ai.h"

namespace Tyr::Soccer
{
Common::Timer activeShootTimer_omid;

void Ai::NewNormalPlay(void)
{
    //	GKHi(gk, 0);
    // side = -side;
    //	GKHi_Simple(gk,0);
    GKHi(gk, 0);
    // side = -side;//TODO #8 comment this ...

    //	DefHi(def,NULL, 0);
    //	side = -side;
    //    DefHi(def,NULL, 0);
    DefMid(def, rw, lw, NULL, false);
    //	side = -side;

    CalcIsDefending();

    if (isDefending)
    {
        NormalPlayDef();
    }
    else
    {
        NormalPlayAtt();
    }
}
} // namespace Tyr::Soccer
