#include <functional>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <signal.h>
#include <stdio.h>

#include <termios.h>
#include <unistd.h>

#include <string>
#include <SDL2/SDL.h>
#include <chrono>

//Main loop flag
bool running = true;

class RawModeConsoleSetter final
{
public:
  RawModeConsoleSetter()
  {
    // get the console in raw mode
    if (tcgetattr(0, &cooked_) < 0)
    {
      throw std::runtime_error("Failed to get old console mode");
    }
    struct termios raw;
    memcpy(&raw, &cooked_, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    raw.c_cc[VTIME] = 1;
    raw.c_cc[VMIN] = 0;
    if (tcsetattr(0, TCSANOW, &raw) < 0)
    {
      throw std::runtime_error("Failed to set new console mode");
    }
  }

  ~RawModeConsoleSetter()
  {
    tcsetattr(0, TCSANOW, &cooked_);
  }

private:
  struct termios cooked_;
};




class QuadControl final
{
public:
  QuadControl()
  {
    nh_ = rclcpp::Node::make_shared("quad_control");
    nh_->declare_parameter("scale_linear", 1.0);

    twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("manual_input", 1); //TODO: change
    //twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("demo/cmd_vel", 1);
  }

  bool initSDLWindow()
  {
    //Initialize SDL
    if(SDL_Init(SDL_INIT_VIDEO) < 0)
    {
      printf("SDL could not initialize! SDL Error: %s\n", SDL_GetError());
      return false;
    }

    //Create window
    gWindow = SDL_CreateWindow("Quadcopter Controller", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if(gWindow == NULL)
    {
      printf("Window could not be created! SDL Error: %s\n", SDL_GetError());
      return false;
    }

    //Get window surface
    gScreenSurface = SDL_GetWindowSurface(gWindow);

    return true;
  }

  bool loadImage(std::string path)
  {
    //Load image at specified path
    loadedSurface = SDL_LoadBMP(path.c_str());
    if(loadedSurface == NULL)
    {
      printf("Unable to load image %s! SDL Error: %s\n", path.c_str(), SDL_GetError());
      return false;
    }
    
    //Apply the current image
    SDL_BlitSurface(loadedSurface, NULL, gScreenSurface, NULL);

    //Update the surface
    SDL_UpdateWindowSurface(gWindow);

    return true;
  }

  void close()
  {
    //Deallocate surface
    SDL_FreeSurface(loadedSurface);
    loadedSurface = NULL;

    //Destroy window
    SDL_DestroyWindow(gWindow);
    gWindow = NULL;

    //Quit SDL subsystems
    SDL_Quit();
  }

  int keyLoop()
  {
    std::thread{std::bind(&QuadControl::spin, this)}.detach();

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the quadcopter on the horizontal plane, W/S to go up/down.");
    puts("'Q' to quit.");


    //Start up SDL and create window
    if(!initSDLWindow())
    {
      printf("Failed to initialize!\n");
    }
    else
    {
      if(!loadImage("include/windowText.bmp"))
      {
        printf("Failed to load media!\n");
      }
      else
      {
        double front_back = 0.0, left_right = 0.0, up_down = 0.0;
        bool arrow_up = false, arrow_down = false, arrow_left = false, arrow_right = false;
        bool key_W = false, key_S = false;

        uint64_t last_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        uint64_t timer_wait_ms = 100; // 100ms --> 10 updates in a sec

        //Event handler
        SDL_Event e;

        //While application is running
        while(running)
        {
          //Handle events on queue
          while(SDL_PollEvent( &e ) != 0)
          {
            switch(e.type){
              //User requests quit
              case SDL_QUIT:
                running = false;
                break;

              //User presses a key
              case SDL_KEYDOWN:
                switch(e.key.keysym.sym){
                  case SDLK_q:
                    running = false;
                    break;

                  case SDLK_w:
                    key_W = true;
                    break;

                  case SDLK_s:
                    key_S = true;
                    break;

                  case SDLK_UP:
                    arrow_up = true;
                    break;

                  case SDLK_DOWN:
                    arrow_down = true;
                    break;

                  case SDLK_LEFT:
                    arrow_left = true;
                    break;

                  case SDLK_RIGHT:
                    arrow_right = true;
                    break;

                  default:
                    break;
                }
                break;

              //User releases a key
              case SDL_KEYUP:
                switch(e.key.keysym.sym){
                  case SDLK_w:
                    key_W = false;
                    break;

                  case SDLK_s:
                    key_S = false;
                    break;

                  case SDLK_UP:
                    arrow_up = false;
                    break;

                  case SDLK_DOWN:
                    arrow_down = false;
                    break;

                  case SDLK_LEFT:
                    arrow_left = false;
                    break;

                  case SDLK_RIGHT:
                    arrow_right = false;
                    break;

                  default:
                    break;
                }
                break;

              default:
                break;
            }


          }

          if((arrow_up && arrow_down) || (!arrow_up && !arrow_down))
            front_back = 0.0;
          else if(arrow_up)
            front_back = 1.0;
          else if(arrow_down)
            front_back = -1.0;

          if((arrow_left && arrow_right) || (!arrow_left && !arrow_right))
            left_right = 0.0;
          else if(arrow_left)
            left_right = 1.0;
          else if(arrow_right)
            left_right = -1.0;

          if((key_W && key_S) || (!key_W && !key_S))
            up_down = 0.0;
          else if(key_W)
            up_down = 1.0;
          else if(key_S)
            up_down = -1.0;


          uint64_t this_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

          if(timer_wait_ms < (this_time - last_time))
          {
            last_time = this_time;
            //printf("Dir: %d, %d, %d.\n", front_back, left_right, up_down);
            geometry_msgs::msg::Twist twist;
            twist.linear.x = nh_->get_parameter("scale_linear").as_double() * front_back;
            twist.linear.y = nh_->get_parameter("scale_linear").as_double() * left_right;
            twist.linear.z = nh_->get_parameter("scale_linear").as_double() * up_down;
            twist_pub_->publish(twist);
          }

          if (abs(front_back) >= 2 || abs(left_right) >= 2 || abs(up_down) >= 2){
            printf("----------ERROR----------: %f, %f, %f\n", front_back, left_right, up_down);
          }
        }
      }
    }

    //Free resources and close SDL
    close();

    return 0;
  }

private:
  void spin()
  {
    rclcpp::spin(nh_);
  }

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  RawModeConsoleSetter console_;

  //The window we'll be rendering to
  SDL_Window* gWindow;

  //The surface contained by the window
  SDL_Surface* gScreenSurface;

  //Text image loaded from bitmap
  SDL_Surface* loadedSurface;

  //Screen dimension constants
  const int SCREEN_WIDTH = 530;
  const int SCREEN_HEIGHT = 95;
};

void quit(int sig)
{
  (void)sig;
  running = false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  signal(SIGINT, quit);

  QuadControl quad_control;

  int rc = quad_control.keyLoop();

  rclcpp::shutdown();

  return rc;
}