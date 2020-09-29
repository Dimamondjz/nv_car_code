#include <cmath>
#include <vector>
#include <string>


float k = 0.1, Lfc = 2.0, Kp = 1.0, dt = 0.1, L=1.0;

struct state
{
    float state_x;
    float state_y;
    float state_yaw;
    float state_v;

    // float state_x = 0.0 ;
    // float state_y = -3.0 ;
    // float state_yaw = 0.0;
    // float state_v = 0.0 ;
};    

struct control
{
    float delta;
    float di;
    int ind;
};

class VehicleState
{
    public:
    VehicleState (float x,float y,float yaw,float v );
    void Update(float a, float delta );
    float PControl(float target, float current);
    control pure_pursuit_control(float cx[], float cy[], int pind, int num_x);
    int calc_target_index(float cx[], float cy[]);
    state state_return;
    // private:
};

template<typename T>
int getArryLen(T& arr) {
    return (sizeof(arr) / sizeof(arr[0]));
}

int sum(float arr[], int len )
{
    int total = 0;
    for (int i = 0; i < len; i++)
    {
        total +=arr[i];
    }
    return total;

}

int VehicleState::calc_target_index(float cx[], float cy[])//会不会出现问题 
{
    float dx[51] = {};
    float dy[51] = {};
    std::vector<float> dist;
    for(int i=0; i< 51; i++)
    {
        dx[i]=state_return.state_x-cx[i];
        // std::cout<<"dx[ "<<i<<" ]"<< dx[i] <<std::endl;
        dy[i]=state_return.state_y-cy[i];
        // std::cout<<"dy[ "<<i<<" ]"<< dy[i] <<std::endl;

    }

    for (int i=0; i<51; i++)
    {
    dist.push_back( fabs( sqrtf( dx[i]*dx[i] + dy[i]*dy[i] ) ) );
    // std::cout<<"distance = "<< fabs( sqrtf( dx[i]*dx[i] + dy[i]*dy[i] ) )<< std::endl;
    }       

    auto dist_index = std::distance(std::begin(dist),std::min_element(std::begin(dist) ,std::end(dist)) );
    // std::cout<<"dist_index = "<<dist_index<< std::endl;
    int ind = dist_index;//后续解决
    L = 0.0;
    float Lf = k * state_return.state_v + Lfc; 
    while (Lf > L && (ind + 1) < 51)
    {
       float L_dx = cx[ind + 1] - cx[ind];
       float L_dy = cy[ind + 1] - cy[ind];
        L += sqrtf(L_dx * L_dx + L_dy * L_dy);
        ind += 1;
    }
    return ind;
}

VehicleState:: VehicleState (float x, float y, float yaw, float v )
{
    state_return.state_x = x;
    // std::cout<<"begin_class.x = " <<state_return.state_x <<std::endl;

    state_return.state_y = y;
    state_return.state_yaw = yaw;
    state_return.state_v = v;
}

void VehicleState::Update(float a, float delta )//这里出现了问题
{
    // std::cout<<"Update  Start " <<std::endl;
    state_return.state_x = state_return.state_x + state_return.state_v * cosf(state_return.state_yaw)* dt;
    state_return.state_y = state_return.state_y + state_return.state_v * sinf(state_return.state_yaw)* dt;
    state_return.state_yaw = state_return.state_yaw + state_return.state_v/L * tanf(delta)* dt;
    // std::cout<<"state_return.state_yaw= *****" << state_return.state_v/L * tanf(delta)* dt <<std::endl;

    state_return.state_v = state_return.state_v + a * dt;
    // std::cout<<"Update.y = " <<state_return.state_y <<std::endl;
    // std::cout<<"Update.v = " <<state_return.state_v <<std::endl;
    // std::cout<<"state_return.state_yaw = " <<state_return.state_yaw <<std::endl;
    // std::cout<<"sinf(state_return.state_yaw) = " << sinf(state_return.state_yaw) <<std::endl;
    // std::cout<<"Update  over " <<std::endl;
    // std::cout<<std::endl;
}

float VehicleState::PControl(float target, float current)
{
    float a =Kp * (target - current);
    return a;
}

control VehicleState::pure_pursuit_control(float cx[], float cy[], int pind ,int num_x)
{
    control pure; 
    pure.ind= calc_target_index(cx, cy); 
    float tx, ty, alpha, Lf;
    if(pind >= pure.ind)
        pure.ind = pind;
    if(pure.ind < 51 )
    {
        tx = cx[pure.ind];
        ty = cy[pure.ind];
    }
    else
    {
        tx = cx[-1];
        ty = cy[-1];
        pure.ind = 51 - 1 ;
    }
    alpha = atan2f(ty - state_return.state_y, tx - state_return.state_x) - state_return.state_yaw;

    if (state_return.state_v < 0)
        alpha = M_PI -alpha;
    Lf = k * state_return.state_v + Lfc;
    pure.delta = atan2f(2.0 * L * sinf(alpha) / Lf, 1.0);
    // std::cout<<"pure.ind = "<<pure.ind << std::endl;
    // std::cout<<"pure.delta = "<<pure.delta << std::endl;
    return pure;
}

