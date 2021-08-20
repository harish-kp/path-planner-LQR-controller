class lqrController{
    public:
    void odom_msg{
        
    }
    int lqrLoop{

    }


}
int main {
    int T = 150; num_steps = 15000; n = 3; m = 2;p = 3;   
    robot = lqrController();
    for (int i = 0; i < num_steps; i++){
        robot.lqrLoop()
    } 
}