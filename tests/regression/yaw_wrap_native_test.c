#include <assert.h>
#include "yaw_wrap.h"
int main(void) {
    assert(Yaw_Wrap360(360)==0);
    assert(Yaw_Wrap360(-360)==0);
    assert(Yaw_Wrap360(361)==1);
    assert(Yaw_Wrap360(-361)==-1);
    assert(Yaw_Wrap360(725)==5);
    assert(Yaw_SampleDelta(1,359)==2);
    assert(Yaw_SampleDelta(-1,-359)==-2);
    assert(Yaw_SampleDelta(359,1)==-2);
    assert(Yaw_SampleDelta(-359,-1)==2);
    float travel=0,prev=350;
    for(int i=1;i<=180;i++){float y=Yaw_Wrap360(350+i);travel+=Yaw_SampleDelta(y,prev);prev=y;}
    assert(travel==180);
    travel=0;prev=-350;
    for(int i=1;i<=180;i++){float y=Yaw_Wrap360(-350-i);travel+=Yaw_SampleDelta(y,prev);prev=y;}
    assert(travel==-180);
    return 0;
}
