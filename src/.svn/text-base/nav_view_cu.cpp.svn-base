/*  Copyright Michael Otte, University of Colorado, 9-9-2009
 *
 *  This file is part of nav_view_cu.
 *
 *  nav_view_cu is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  nav_view_cu is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with nav_view_cu. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  If you require a different license, contact Michael Otte at
 *  michael.otte@colorado.edu
 *
 *  nav_view_cu is a minimalistic GUI for 2D planning that is a stand-in 
 *  for the nav_view node in the default ROS installation. We created 
 *  nav_view_cu for use on slower computers and computers without GPUs 
 *  (the normal nav_view relies on the Ogre graphics package, which 
 *  requires a GPU).
 *
 *  nav_view_cu requires that you have the opengl and glut packages installed. 
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>

#include </usr/include/GL/glut.h>	    // GLUT
#include </usr/include/GL/glu.h>	    // GLU
#include </usr/include/GL/gl.h>	        // OpenGL

#include <ros/ros.h>

#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point32.h"

#include "nav_msgs/Path.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/GridCells.h"

#include "sensor_msgs/PointCloud.h"

#include "move_base_msgs/MoveBaseActionGoal.h"

#include <list>

#ifndef PI
  #define PI 3.1415926535897
#endif

#define SCANNER_RANGE 5.5 // the range of the scanner in meters;
        
struct MAP;
typedef struct MAP MAP;

struct POSE;
typedef struct POSE POSE;

struct POINT_LIST;
typedef struct POINT_LIST POINT_LIST;

struct GRID_LIST;
typedef struct GRID_LIST GRID_LIST;

struct ROBOT;
typedef struct ROBOT ROBOT;

// colors
float WHITE[] = {1,1,1,1};
float BLACK[] = {0,0,0,1};
float DARK[] = {.1,.1,.1,1};
float LIGHTGRAY[] = {.8,.8,.8,1};
float LIGHT[] = {.9,.9,.9,1};
float RED[] = {1,0,0,1};
float BLUE[] = {0,0,1,1};
float GREEN[] = {0,1,0,1};

// globals used for keeping track of display attributes
int display_size_init = 600; // pixels
int menu_pixels = 35;
int win_width = 1;
int win_height = 1;
float win_pan_x = 0;
float win_pan_y = 0;
float win_aspect_ratio = 1;
float scale_factor = 1; // radius of the size of the word
float max_grids_on_side = 500;
float robot_display_radius = .3;
float laser_hit_display_rad = .02;
int display_flag = 1; // set to 0 to avoid drawing, set to 1 to draw
int menu_flag  = 1; // same as above, but only for menu
float map_x_offset = 0;
float map_y_offset = 0;

// display state
enum Display_State {MOVE, SETGOAL, SETPOSE, RELOADMAP, MOVEROBOT, UPRES, DOWNRES, NONE};
Display_State display_state = RELOADMAP;
Display_State mouseover_state = NONE;
Display_State mousedown_state = NONE;

// globals used for user interaction
int left_pressed = 0;
int right_pressed = 0;
float mouse_pan_x_last;
float mouse_pan_y_last;
float mouse_zoom_y_init;
float scale_factor_init;
float new_location_x;
float new_location_y;
float new_heading_x;
float new_heading_y;

// globals used for map etc.
MAP* costmap = NULL;
ROBOT* robot = NULL;
POINT_LIST* global_path = NULL;
POINT_LIST* local_path = NULL;
GRID_LIST* inflated_obs_local = NULL;
GRID_LIST* obstacles_local = NULL;

// global ROS subscriber handles
ros::Subscriber particle_cloud_sub;
ros::Subscriber robot_footprint_sub; 
ros::Subscriber global_path_sub;
ros::Subscriber local_path_sub;
ros::Subscriber inflated_obs_local_sub;
ros::Subscriber obstacles_local_sub;


// global ROS publisher handles
ros::Publisher goal_pub;
ros::Publisher new_pose_pub;

// other ROS globals
ros::Rate loop_rate(100);
        
// globals used to set user robot control
double change_speed = 0; // 0 = no, -1 = decrease speed, 1 = increase speed
double change_turn = 0; // 0 = no, -1 = decrease turn, 1 = increase turn

/*-------------------- basic drawing functions --------------------------*/

int max(int a, int b)
{
  if(a > b)
    return a;
  return b;
}

// draws a rectangle
void drawRectangle(float* pos, float* size, float* color)
{
  glPushMatrix();
    glTranslatef(pos[0],pos[1],pos[2]);
    glScaled(size[0],size[1],1); 
   
    glBegin(GL_POLYGON);
      glColor3f(color[0],color[1],color[2]); 
        glVertex2f(0,1);
        glVertex2f(1,1);
        glVertex2f(1,0);
        glVertex2f(0,0);
    glEnd(); 
   
  glPopMatrix();
}

// draws a rectangle outline
void drawRectangleOutline(float* pos, float* size, float* color)
{
  glPushMatrix();
    glTranslatef(pos[0],pos[1],pos[2]);
    glScaled(size[0],size[1],1); 
   
    glBegin(GL_LINE_STRIP);
      glColor3f(color[0],color[1],color[2]); 
        glVertex2f(0,1);
        glVertex2f(1,1);
        glVertex2f(1,0);
        glVertex2f(0,0);
        glVertex2f(0,1);
    glEnd(); 
   
  glPopMatrix();
}

// draws an arrow from pos1 to pos2
void draw_arrow(float* pos1, float* pos2, float* color)
{
  glColor3f(color[0],color[1],color[2]);
  glBegin(GL_LINE_STRIP);
    glVertex3f(pos1[0],pos1[1],pos1[2]);
    glVertex3f(pos2[0],pos2[1],pos2[2]);
  glEnd();
}

void draw_circle(float* pos, float rad, float* color)
{
  glPushMatrix();
  glTranslatef(pos[0], pos[1], pos[2]);
  glScaled(rad, rad, 1);  
  glColor3f(color[0],color[1],color[2]);
  glBegin(GL_LINE_LOOP); 
  float points = 20;
  for (float i = 0; i < points; i++)
  {    
    float a = 2*PI*i/points; 
    glVertex2f(cos(a), sin(a)); 
  } 
  glEnd();
  glPopMatrix();
}

// outputs the text
void draw_string(float* pos, float pad_left, float pad_bottom, const char* string) 
{
    glRasterPos3f(pos[0]+pad_left, pos[1]+pad_bottom, pos[2]);
    for(int i = 0; string[i] != '\0'; i++)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, string[i]);
}

/*---------------------- MAP --------------------------------------------*/
struct MAP
{
    float** cost;
    int height;
    int width;
    float resolution;
};

// this creats and returns a pointer to a map struct
MAP* make_map(int height, int width, float resolution)
{
  MAP* map = (MAP*)calloc(1, sizeof(MAP));
  map->height = height;
  map->width = width;
  map->resolution = resolution; 
  
  map->cost = (float**)calloc(height, sizeof(float*));
        
  for (int y = 0; y < height; y++)  
  {
	map->cost[y] = (float*)calloc(width, sizeof(float));  
  } 
  return map;
}


// this allocates all required memory for a map
void destroy_map(MAP* map)
{
  int y;
  if(map != NULL)
  {
    for (y = 0; y < map->height; y++)  
    {
	  if(map->cost[y] != NULL)
        free(map->cost[y]);

    }

    if(map->cost != NULL)
      free(map->cost);

    map->height = 0;
    map->width = 0;
    free(map);
  }
}

// prints map on command line
void print_map(MAP* map)
{
  int y, x;

  if(map->cost != NULL)
  {
    printf("\n");  
    for (y = 0; y < map->height; y++)  
    {
      for (x = 0; x < map->width; x++)  
	    printf(" %f", map->cost[y][x]);
      printf("\n");  
    }
    printf("\n");  
  } 
}

MAP* load_map()
{
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...\n");
  if( !ros::service::call("/static_map", req, resp) )
  {
    ROS_INFO("request failed\n");
    return NULL;
  }
  
  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n", resp.map.info.width, resp.map.info.height, resp.map.info.resolution);

  if (resp.map.info.width == 0 && resp.map.info.height == 0)
    return NULL;

  float map_resolution = resp.map.info.resolution;
  int map_width = resp.map.info.width;
  int map_height = resp.map.info.height;

  MAP* map = make_map(map_height, map_width, map_resolution);

  
  float x = resp.map.info.origin.position.x; 
  float y = resp.map.info.origin.position.y;
  float z = resp.map.info.origin.position.z;
          
  float qx = resp.map.info.origin.orientation.x;
  float qy = resp.map.info.origin.orientation.y;
  float qz = resp.map.info.origin.orientation.z;
  float qw = resp.map.info.origin.orientation.w;
          
  ROS_INFO("pose offset: [%f %f %f] \n orientation offset: [%f %f %f %f] \n",x,y,z,qx,qy,qz,qw);
  
  if(z != 0 || qx != 0 || qy != 0 || qz != 0 || qw != 1)
      ROS_ERROR("Map Orientation or Height offset is not supported \n");
  
  map_x_offset = x;
  map_y_offset = y;
  
  for(int i = 0; i < map_height; i++)
  {
    for(int j = 0; j < map_width; j++)
    {
      if(resp.map.data[i*map_width+j] == 100)
        map->cost[i][j] = 0;
      else if(resp.map.data[i*map_width+j] == 0)
        map->cost[i][j] = 1;
      else
        map->cost[i][j] = .5;
    }
  }
  return map;
}

MAP* load_map_debug()
{ 
  int map_height = 6;
  int map_width = 10;
  float map_resolution = 1;
  
  MAP* map = make_map(map_height, map_width, map_resolution);
     
  for(int i = 0; i < map_height; i++)
  {
    for(int j = 0; j < map_width; j++)
    {
      if(i == j)
        map->cost[i][j] = 0;
      else if(i > j)
        map->cost[i][j] = 1;
      else
        map->cost[i][j] = .5;
    }
  }
  return map;
}

// creates a blank map of height, width, and resolution
MAP* load_blank_map(int map_height, int map_width, float map_resolution)
{
  MAP* map = make_map(map_height, map_width, map_resolution);
     
  for(int i = 0; i < map_height; i++)
    for(int j = 0; j < map_width; j++)
    {
        map->cost[i][j] = 1;   
    }
  return map;
}


// draws the map in the gui at buffer height z_height
void draw_map(MAP* map, float z_height)
{
  glPushMatrix(); 
    
  glTranslatef(-1, -1, z_height);
  float map_rad = 2/(float)max(map->height, map->width); 
  glScaled(map_rad,map_rad,1);
    
  int y, x;
  if(map->cost != NULL)
  {
    glBegin(GL_QUADS);
    
    int height = map->height;
    int width = map->width;
    float** cost = map->cost;
    
    for (y = 0; y < height; y++)  
    {
      float float_y = (float)y;
      for (x = 0; x < width; x++)  
      {
        float float_x = (float)x;
	    
        glColor3f(cost[y][x], cost[y][x], cost[y][x]); 
          glVertex2f(float_x, float_y + 1);
          glVertex2f(float_x + 1, float_y + 1);
          glVertex2f(float_x + 1, float_y);
          glVertex2f(float_x, float_y);    
      }
    }
    glEnd(); 
  } 
  glPopMatrix();
}


// draws a subsampled version of the map in the gui at buffer height z_height
void draw_coarse_map(MAP* map, float z_height, float grids_per_rad)
{
  glPushMatrix(); 
    
  glTranslatef(-1, -1, z_height);
  float map_rad = 2/(float)max(map->height, map->width); 
  glScaled(map_rad,map_rad,1);
  
  float grids_per_rect = (float)((int)((float)max(map->height, map->width))/grids_per_rad);  
  
  if(display_state == MOVE && mousedown_state == NONE && (left_pressed == 1 || right_pressed == 1))
    grids_per_rect = grids_per_rect*4;
    
  if(grids_per_rect < 1)
    grids_per_rect = 1;
  
  if(grids_per_rect > (float)max(map->height, map->width))
      grids_per_rect = (float)max(map->height, map->width);
  
  glTranslatef(-1, -1, 0);  
  
  
  if(map->cost != NULL)
  {   
    // plot min (darkest) of grids that are combined into a single rectangle
    glBegin(GL_QUADS);
    
    int height = map->height;
    int width = map->width;
    float** cost = map->cost;
    
    int x, y, x_, y_, next_y, next_x;
    float val;
    for(y = 0; y < height; y += grids_per_rect)  
    {
      next_y = y+grids_per_rect;
      if(next_y > height)
        next_y = height;
        
      for(x = 0; x < width; x += grids_per_rect)  
      {  
        next_x = x+grids_per_rect;
        if(next_x > width)
          next_x = width;    
          
        val = 1;
       
        for(y_ = y; y_ < next_y; y_++) 
          for(x_ = x; x_ < next_x; x_++) 
            if(val > cost[y_][x_])
              val = cost[y_][x_];   
          
        float float_x = (float)x;
        float float_y = (float)y;
        
        glColor3f(val, val, val); 
          glVertex2f(float_x, float_y + grids_per_rect);
          glVertex2f(float_x + grids_per_rect, float_y + grids_per_rect);
          glVertex2f(float_x + grids_per_rect, float_y);
          glVertex2f(float_x, float_y);         
      }
    }
    glEnd(); 
  } 
  glPopMatrix();
}

/* ----------------------- POSE -----------------------------------------*/
struct POSE
{
    float x;
    float y;
    float z;
    
    float qw;
    float qx;
    float qy;
    float qz; 
    
    float cos_alpha;
    float sin_alpha;
    
};

// this creates and returns a pointer to a POSE struct
POSE* make_pose(float x, float y, float z)
{
  POSE* pose = (POSE*)calloc(1, sizeof(POSE));
  pose->x = x;
  pose->y = y;
  pose->z = z; 
 
  pose->qw = 1;
  pose->qx = 0;
  pose->qy = 0;
  pose->qz = -1;  
  
  pose->cos_alpha = 0;
  pose->sin_alpha = 1;
  
  return pose;
}


// this allocates all required memory for a POSE
void destroy_pose(POSE* pose)
{
  if(pose != NULL)
    free(pose);
}

// prints pose on command line
void print_pose(POSE* pose)
{
  if(pose != NULL)
  {
    printf("\n");  
    printf("x: %f, y: %f, z: %f \n",pose->x, pose->y, pose->z);  
    printf("\n");  
  } 
}

// draws the pose at z_height with color clr
void draw_pose(POSE* pose, float* clr, float z_height)
{
  // now draw pose circle and directional arrow 
  if((pose != NULL) & (costmap!= NULL))
  { 
    glPushMatrix(); 
    glTranslatef(-1, -1, z_height);
  
    if(costmap != NULL)
    {
      float map_rad = 2/(float)max(costmap->height, costmap->width); 
      glScaled(map_rad,map_rad,1);
    }
    
    float scaled_rad = robot_display_radius/costmap->resolution;
    float pos[] = {pose->x, pose->y, 0};
    draw_circle(pos, scaled_rad , clr);
    
    float pos2[] = {pose->x + scaled_rad*pose->cos_alpha, pose->y + scaled_rad*pose->sin_alpha, 0};
    draw_arrow(pos, pos2, clr);
    
    glPopMatrix(); 
  }
  
}
/*----------------------- POINT_LIST ------------------------------------*/

struct POINT_LIST
{
    float** points;
    int length;
};

// this creats and returns a pointer to a map struct
POINT_LIST* make_point_list(int length)
{
  POINT_LIST* pl = (POINT_LIST*)calloc(1, sizeof(POINT_LIST));
  pl->length = length; 
  
  pl->points = (float**)calloc(length, sizeof(float*));
        
  for (int y = 0; y < length; y++)  
  {
	pl->points[y] = (float*)calloc(3, sizeof(float));  
  } 
  
  return pl;
}


// this deallocates all required memory for a POINT_LIST
void destroy_point_list(POINT_LIST* pl)
{
  int y;
  if(pl != NULL)
  {
    for (y = 0; y < pl->length; y++)  
    {
	  if(pl->points[y] != NULL)
        free(pl->points[y]);
    }

    if(pl->points != NULL)
      free(pl->points);
    
    pl->length = 0;
    
    free(pl);
  }
}

// prints point list on command line
void print_point_list(POINT_LIST* pl)
{
  int y;

  if(pl->points != NULL)
  {
    printf("\n");  
    for (y = 0; y < pl->length; y++)  
    {
	    printf(" %f, %f, %f", pl->points[y][0], pl->points[y][1], pl->points[y][2]);
      printf("\n");  
    }
    printf("\n");  
  } 
}

// draws pl in the gui at buffer height z_height as line segments, if connect = true, then connect first and last points
void draw_point_list_2D_lines(POINT_LIST* pl, float* color, float z_height, bool connect)
{    
  glPushMatrix(); 
    
  glTranslatef(-1, -1, z_height);
  
  if(costmap != NULL)
  {
    float map_rad = 2/(float)max(costmap->height, costmap->width); 
    glScaled(map_rad,map_rad,1);
  }
  
  int y;
  if((pl != NULL) & (pl->points != NULL))
  {
    glBegin(GL_LINE_STRIP);
    glColor3f(color[0], color[1], color[2]); 
    
    float** points = pl->points;
    int length = pl->length;
    
    for (y = 0; y < length; y++)  
      glVertex2f(points[y][0], points[y][1]);
    
    if(connect && 0 < length)
      glVertex2f(points[0][0], points[0][1]);     
    
    glEnd(); 
  } 
  glPopMatrix();
}

// draws pl in the gui at buffer height z_height as grids, 1 grid of size rad centered on each point
void draw_point_list_grids(POINT_LIST* pl, float* color, float rad, float z_height)
{    
  glPushMatrix(); 
    
  glTranslatef(-1, -1, z_height);
  
  if(costmap != NULL)
  {
    float map_rad = 2/(float)max(costmap->height, costmap->width); 
    glScaled(map_rad,map_rad,1);
  }
  
  int y;
  if((pl != NULL) & (pl->points != NULL))
  {
    glBegin(GL_QUADS);
    glColor3f(color[0], color[1], color[2]); 
    
    float** points = pl->points;
    int length = pl->length;
    float height = rad/costmap->resolution;
    float width = rad/costmap->resolution;
    
    for (y = 0; y < length; y++)  
    {
      glVertex2f(points[y][0] - width, points[y][1] + height);
      glVertex2f(points[y][0] + width, points[y][1] + height);
      glVertex2f(points[y][0] + width, points[y][1] - height);
      glVertex2f(points[y][0] - width, points[y][1] - height);  
    }
    glEnd(); 
  } 
  glPopMatrix();
}

// draws pl in the gui at buffer height z_height as grids, 1 grid of size rad centered on each point
// uses z of the points to denote color1 or color2 if it is 0 or 1, respectively
void draw_point_list_grids_binary(POINT_LIST* pl, float* color1, float* color2, float rad, float z_height)
{    
  glPushMatrix(); 
    
  glTranslatef(-1, -1, z_height);
  
  if(costmap != NULL)
  {
    float map_rad = 2/(float)max(costmap->height, costmap->width); 
    glScaled(map_rad,map_rad,1);
  }
  
  int y;
  if((pl != NULL) & (pl->points != NULL))
  {
    glBegin(GL_QUADS);
    
    float** points = pl->points;
    int length = pl->length;
    float height = rad/costmap->resolution;
    float width = rad/costmap->resolution;
    
    for (y = 0; y < length; y++)  
    {
      if(points[y][2] == 0)
        glColor3f(color1[0], color1[1], color1[2]);   
      else
        glColor3f(color2[0], color2[1], color2[2]);   
      
      glVertex2f(points[y][0] - width, points[y][1] + height);
      glVertex2f(points[y][0] + width, points[y][1] + height);
      glVertex2f(points[y][0] + width, points[y][1] - height);
      glVertex2f(points[y][0] - width, points[y][1] - height);  
    }
    glEnd(); 
  } 
  glPopMatrix();
}

/*----------------------- GRID_LIST ------------------------------------*/

struct GRID_LIST
{
    POINT_LIST* grids;
    float height;
    float width;
};

// this creates and returns a pointer to a map struct
GRID_LIST* make_grid_list(int length, float height, float width)
{
  GRID_LIST* gl = (GRID_LIST*)calloc(1, sizeof(GRID_LIST));

  gl->grids = make_point_list(length);
  gl->height = height;
  gl->width = width;
  
  return gl;
}


// this deallocates all required memory for a GRID_LIST
void destroy_grid_list(GRID_LIST* gl)
{
  if(gl != NULL)
  {
    destroy_point_list(gl->grids);
    free(gl);
  }
}


// draws gl in the gui at buffer height z_height as quads
void draw_grid_list_2D_quads(GRID_LIST* gl, float* color, float z_height)
{    
  glPushMatrix(); 
    
  glTranslatef(-1, -1, z_height);
  
  if(costmap != NULL)
  {
    float map_rad = 2/(float)max(costmap->height, costmap->width); 
    glScaled(map_rad,map_rad,1);
  }
  
  int y;
  if((gl != NULL) & (gl->grids != NULL) & (gl->grids->points != NULL))
  {
    glBegin(GL_QUADS);
    glColor3f(color[0], color[1], color[2]); 
    
    float** points = gl->grids->points;
    int length = gl->grids->length;
    float height = gl->height/2;
    float width = gl->width/2;
    
    for (y = 0; y < length; y++)  
    {
      glVertex2f(points[y][0] - width, points[y][1] + height);
      glVertex2f(points[y][0] + width, points[y][1] + height);
      glVertex2f(points[y][0] + width, points[y][1] - height);
      glVertex2f(points[y][0] - width, points[y][1] - height);  
    }
    glEnd(); 
  } 
  glPopMatrix();
}

/*------------------------ ROBOT ----------------------------------------*/
struct ROBOT
{
  POINT_LIST* bound;
  POSE* pose;
  float color[3];
};

// this creates and returns a pointer to a ROBOT struct
ROBOT* make_robot(float x, float y, float z, float* color)
{
  ROBOT* bot = (ROBOT*)calloc(1, sizeof(ROBOT));
  
  bot->bound = make_point_list(0);
  bot->pose = make_pose(x, y, z);
 
  bot->color[0] = color[0];
  bot->color[1] = color[1];
  bot->color[2] = color[2];
  
  return bot;
}

// this deallocates all required memory for a robot
void destroy_robot(ROBOT* bot)
{
  destroy_pose(bot->pose);
  destroy_point_list(bot->bound);
  
  if(bot != NULL)
    free(bot);
}

// draws the robot in the gui at buffer height z_height as quads
void draw_robot(ROBOT* bot, float z_height)
{    
  if(bot == NULL) 
    return;
          
  // draw the outline of the robot
  if(bot->bound != NULL)
    draw_point_list_2D_lines(bot->bound, bot->color, z_height, true);
    
  // now draw pose circle and directional arrow 
  if((bot->pose != NULL) & (costmap!= NULL))
  { 
    glPushMatrix(); 
    glTranslatef(-1, -1, z_height);
  
    if(costmap != NULL)
    {
      float map_rad = 2/(float)max(costmap->height, costmap->width); 
      glScaled(map_rad,map_rad,1);
    }
    
    float scaled_rad = robot_display_radius/costmap->resolution;
    float pos[] = {bot->pose->x, bot->pose->y, 0};
    draw_circle(pos, scaled_rad , GREEN);
    
    float pos2[] = {bot->pose->x + scaled_rad*bot->pose->cos_alpha, bot->pose->y + scaled_rad*bot->pose->sin_alpha, 0};
    draw_arrow(pos, pos2, RED);
    
    glPopMatrix(); 
  }
}

/*-------------------------- MENU ---------------------------------------*/

//draws the menu at buffer height z_height
void draw_menu(float z_height)
{
  glDepthFunc(GL_ALWAYS);
  
  float menu_bottom = ((float)win_height/2 - (float)menu_pixels)/((float)win_height/2);
  float menu_height = 1 - menu_bottom;
  float pxls = menu_height/(float)menu_pixels;
  float menu_width = (float)win_width*pxls;
  
  glScaled(scale_factor, scale_factor, 1);
  glTranslatef(-menu_width/2, menu_bottom, z_height);  
 
  float pos[] = {0, 0, 0};
  float size[] = {menu_width, menu_height};
  drawRectangle(pos, size, LIGHT);
  
  float size_button[] = {90*pxls, 25*pxls};
  float size_small_button[] = {40*pxls, 25*pxls};
  
  float pos_move_button[] = {5*pxls, 5*pxls, 0};
  if(mouseover_state == MOVE)
  {
    drawRectangle(pos_move_button, size_button, (left_pressed == 1) ? LIGHTGRAY : WHITE);
    drawRectangleOutline(pos_move_button, size_button, BLACK);
  }
  else if(display_state == MOVE)
  {
    drawRectangle(pos_move_button, size_button, LIGHTGRAY);
    drawRectangleOutline(pos_move_button, size_button, BLACK);
  }
  glColor3f(0, 0, 0);
  draw_string(pos_move_button, 15*pxls, 8*pxls, "Move Map");

  
  float pos_goal_button[] = {105*pxls, 5*pxls, 0};
  if(mouseover_state == SETGOAL)
  {
    drawRectangle(pos_goal_button, size_button, (left_pressed == 1) ? LIGHTGRAY : WHITE);
    drawRectangleOutline(pos_goal_button, size_button, BLACK);   
  }
  else if(display_state == SETGOAL)
  {
    drawRectangle(pos_goal_button, size_button, LIGHTGRAY);
    drawRectangleOutline(pos_goal_button, size_button, BLACK); 
  }
  draw_string(pos_goal_button, 20*pxls, 8*pxls, "Set Goal");
  
  
  float pos_pose_button[] = {205*pxls, 5*pxls, 0};
  if(mouseover_state == SETPOSE)
  {
    drawRectangle(pos_pose_button, size_button, (left_pressed == 1) ? LIGHTGRAY : WHITE);
    drawRectangleOutline(pos_pose_button, size_button, BLACK);  
  }
  else if(display_state == SETPOSE)
  {
    drawRectangle(pos_pose_button, size_button, LIGHTGRAY);
    drawRectangleOutline(pos_pose_button, size_button, BLACK);  
  }
  draw_string(pos_pose_button, 20*pxls, 8*pxls, "Set Pose");
  
  
  float pos_reload_button[] = {305*pxls, 5*pxls, 0};
  if(mouseover_state == RELOADMAP)
  {
    drawRectangle(pos_reload_button, size_button, (left_pressed == 1) ? LIGHTGRAY : WHITE);
    drawRectangleOutline(pos_reload_button, size_button, BLACK);  
  }
  else if(display_state == RELOADMAP)
  {
    drawRectangle(pos_reload_button, size_button, LIGHTGRAY);
    drawRectangleOutline(pos_reload_button, size_button, BLACK); 
  }
  draw_string(pos_reload_button, 12*pxls, 8*pxls, "Reload Map");
  
  /*
  float move_robot_button[] = {405*pxls, 5*pxls, 0};
  if(mouseover_state == MOVEROBOT)
  {
    drawRectangle(move_robot_button, size_button, (left_pressed == 1) ? LIGHTGRAY : WHITE);
    drawRectangleOutline(move_robot_button, size_button, BLACK);  
  }
  else if(display_state == MOVEROBOT)
  {
    drawRectangle(move_robot_button, size_button, LIGHTGRAY);
    drawRectangleOutline(move_robot_button, size_button, BLACK); 
  }
  draw_string(move_robot_button, 10*pxls, 8*pxls, "Move Robot");
  */
  
  float up_res_button[] = {505*pxls, 5*pxls, 0};
  if(mouseover_state == UPRES)
  {
    drawRectangle(up_res_button, size_small_button, (left_pressed == 1) ? LIGHTGRAY : WHITE);
    drawRectangleOutline(up_res_button, size_small_button, BLACK);  
  }
  else if(display_state == UPRES)
  {
    drawRectangle(up_res_button, size_small_button, LIGHTGRAY);
    drawRectangleOutline(up_res_button, size_small_button, BLACK); 
  }
  draw_string(up_res_button, 5*pxls, 8*pxls, "Res+");
  
  float down_res_button[] = {555*pxls, 5*pxls, 0};
  if(mouseover_state == DOWNRES)
  {
    drawRectangle(down_res_button, size_small_button, (left_pressed == 1) ? LIGHTGRAY : WHITE);
    drawRectangleOutline(down_res_button, size_small_button, BLACK);  
  }
  else if(display_state == DOWNRES)
  {
    drawRectangle(down_res_button, size_small_button, LIGHTGRAY);
    drawRectangleOutline(down_res_button, size_small_button, BLACK); 
  }
  draw_string(down_res_button, 5*pxls, 8*pxls, "Res-");
  
  glDepthFunc(GL_LESS);
}

/*----------------------- ROS Callbacks ---------------------------------*/
void particle_cloud_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  // currently, just use the first particle in the cloud as the robot position
  float x_scl = 1/costmap->resolution;
  float y_scl = 1/costmap->resolution;
  float z_scl = 0;
  
  destroy_pose(robot->pose);
  robot->pose = make_pose((msg->poses[0].position.x-map_x_offset)*x_scl, (msg->poses[0].position.y-map_y_offset)*y_scl, msg->poses[0].position.z*z_scl);
  
  float qw = msg->poses[0].orientation.w;
  float qx = msg->poses[0].orientation.x;
  float qy = msg->poses[0].orientation.y;
  float qz = msg->poses[0].orientation.z; 
  
  robot->pose->qw = qw;
  robot->pose->qx = qx;
  robot->pose->qy = qy;
  robot->pose->qz = qz;  
  
  robot->pose->cos_alpha = qw*qw + qx*qx - qy*qy - qz*qz;
  robot->pose->sin_alpha = 2*qw*qz + 2*qx*qy; 
  
  display_flag = 1;
  glutPostRedisplay(); 
}

void robot_footprint_callback(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
  if(costmap == NULL)
    return;
     
  float x_scl = 1/costmap->resolution;
  float y_scl = 1/costmap->resolution;
  float z_scl = 0;
    
  int size = msg->polygon.points.size();
  destroy_point_list(robot->bound);
  robot->bound = make_point_list(size);

  for(int i = 0; i < size; i++)
  {
    robot->bound->points[i][0] = (msg->polygon.points[i].x-map_x_offset)*x_scl;
    robot->bound->points[i][1] = (msg->polygon.points[i].y-map_y_offset)*y_scl;
    robot->bound->points[i][2] = msg->polygon.points[i].z*z_scl; 
  }
 
  display_flag = 1;
  glutPostRedisplay(); 
 
  //ROS_INFO("New Robot Bound: \n");
  //print_point_list(robot->bound);
}

void global_plan_callback(const nav_msgs::Path::ConstPtr& msg)
{     
  if(costmap == NULL)
   return;
    
  float x_scl = 1/costmap->resolution;
  float y_scl = 1/costmap->resolution;
  float z_scl = 0; 
  
  int length = msg->poses.size();
  destroy_point_list(global_path);
  global_path = make_point_list(length);
  
  for(int i = 0; i < length; i++)
  {
    global_path->points[i][0] = x_scl*(msg->poses[i].pose.position.x-map_x_offset);
    global_path->points[i][1] = y_scl*(msg->poses[i].pose.position.y-map_y_offset);
    global_path->points[i][2] = z_scl*msg->poses[i].pose.position.z;   
  }

  display_flag = 1;
  glutPostRedisplay(); 
  
  //ROS_INFO("New Global Path: \n");
  //print_point_list(global_path); 
}

void local_plan_callback(const nav_msgs::Path::ConstPtr& msg)
{     
  if(costmap == NULL)
    return;
     
  float x_scl = 1/costmap->resolution;
  float y_scl = 1/costmap->resolution;
  float z_scl = 0; 
  
  int length = msg->poses.size();
  destroy_point_list(local_path);
  local_path = make_point_list(length);
  
  for(int i = 0; i < length; i++)
  {
    local_path->points[i][0] = x_scl*(msg->poses[i].pose.position.x-map_x_offset);
    local_path->points[i][1] = y_scl*(msg->poses[i].pose.position.y-map_y_offset);
    local_path->points[i][2] = z_scl*msg->poses[i].pose.position.z;   
  }

   display_flag = 1;
   glutPostRedisplay(); 
}

void inflated_obs_local_callback(const nav_msgs::GridCells::ConstPtr& msg)
{     
  if(costmap == NULL)
    return;
    
  float x_scl = 1/costmap->resolution;
  float y_scl = 1/costmap->resolution;
  float z_scl = 0; 
  
  float width = x_scl*msg->cell_width;
  float height = x_scl*msg->cell_height;        
  int length = msg->cells.size();
  
  destroy_grid_list(inflated_obs_local);
  inflated_obs_local = make_grid_list(length, height, width);
  
  for(int i = 0; i < length; i++)
  {
    inflated_obs_local->grids->points[i][0] = x_scl*(msg->cells[i].x-map_x_offset);
    inflated_obs_local->grids->points[i][1] = y_scl*(msg->cells[i].y-map_y_offset);
    inflated_obs_local->grids->points[i][2] = z_scl*msg->cells[i].z;   
  }

   display_flag = 1;
   glutPostRedisplay(); 
  
  //ROS_INFO("New Inflated Obstacle List: \n");
  //print_point_list(inflated_obs_local->grids);
  
}


void obstacles_local_callback(const nav_msgs::GridCells::ConstPtr& msg)
{      
 if(costmap == NULL)
   return;
    
  float x_scl = 1/costmap->resolution;
  float y_scl = 1/costmap->resolution;
  float z_scl = 0; 
  
  float width = x_scl*msg->cell_width;
  float height = x_scl*msg->cell_height;        
  int length = msg->cells.size();
  
  destroy_grid_list(obstacles_local);
  obstacles_local = make_grid_list(length, height, width);
  
  for(int i = 0; i < length; i++)
  {
    obstacles_local->grids->points[i][0] = x_scl*(msg->cells[i].x-map_x_offset);
    obstacles_local->grids->points[i][1] = y_scl*(msg->cells[i].y-map_y_offset);
    obstacles_local->grids->points[i][2] = z_scl*msg->cells[i].z;   
  }

   display_flag = 1;
   glutPostRedisplay(); 
  
  //ROS_INFO("New Inflated Obstacle List: \n");
  //print_point_list(inflated_obs_local->grids); 
}

/*----------------------- GLUT Callbacks --------------------------------*/

// this sets the projection in the current window
void Project()
{
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   glOrtho(-win_aspect_ratio*scale_factor,+win_aspect_ratio*scale_factor, -scale_factor,+scale_factor, -1,+1);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
}

// default display function, gets called once each loop through glut
void display()
{   
            
   // set up graphics stuff
   Project();

   if(display_flag == 1)  
   {  
     glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
     glEnable(GL_DEPTH_TEST);
     glDisable (GL_BLEND);
     glCullFace(GL_BACK);  
     glLoadIdentity();
     glPushMatrix();
     glDepthFunc(GL_ALWAYS);  
     glTranslatef(win_pan_x,win_pan_y,0);
   
     // draw the map
     if(costmap != NULL)
     {
       draw_coarse_map(costmap, -.98, max_grids_on_side);        
       //draw_map(costmap, -.98);
     }
   
     // draw inflated local obstacles
     if(inflated_obs_local != NULL)
     {
       draw_grid_list_2D_quads(inflated_obs_local, BLUE, -.97);
     }
   
     // draw local obstacles
     if(obstacles_local != NULL)
     {
       draw_grid_list_2D_quads(obstacles_local, RED, -.96);
     }
     
     // draw the global path
     if(global_path != NULL)
     {
       draw_point_list_2D_lines(global_path, GREEN, .97, false);
     }
   
     // draw the local path
     if(local_path != NULL)
     {
       draw_point_list_2D_lines(local_path, BLUE, .975, false);
     }
     
     // draw the robot
     if(robot != NULL)
     {
        draw_robot(robot, .98);
     }
   
     // draw arrows for setting pose and goal
     if(display_state == SETGOAL && left_pressed)
     {
        float arrow_start[] =  {new_location_x, new_location_y, .99};
        float arrow_end[] =  {new_location_x+new_heading_x, new_location_y+new_heading_y, .99};
        draw_arrow(arrow_start, arrow_end, GREEN);       
     }
     else if(display_state == SETPOSE && left_pressed)
     {
        float arrow_start[] =  {new_location_x, new_location_y, .99};
        float arrow_end[] =  {new_location_x+new_heading_x, new_location_y+new_heading_y, .99};
        draw_arrow(arrow_start, arrow_end, BLUE);     
     }
         
     glPopMatrix();
     glDepthFunc(GL_LESS);
     
     display_flag = 0; 
     menu_flag = 1;
   }
   
   if(menu_flag == 1)
   {
     // draw the menu
     draw_menu(.995);
     menu_flag = 0;
   }
   
   // draw the updated scene
   glFlush();
   glutSwapBuffers(); 
}


// this routine is called when the window is resized
void reshape(int width,int height)
{
   win_width = width;
   win_height = height;
   //  Ratio of the width to the height of the window
   win_aspect_ratio = (height>0) ? (double)width/height : 1;
   //  Set the viewport to the entire window
   glViewport(0,0, width, height);
   //  Set projection
   Project();
   
   display_flag = 1;
   glutPostRedisplay();   
}

// this routine gets called when special keys are pressed
void special(int key,int x,int y)
{
  if(display_state == MOVE)
  {
    if (key == GLUT_KEY_RIGHT)
    {
      win_pan_x -= .1*scale_factor;
      display_flag = 1; 
      glutPostRedisplay(); 
    }
    else if (key == GLUT_KEY_LEFT)
    {
      win_pan_x += .1*scale_factor;
      display_flag = 1; 
      glutPostRedisplay(); 
    }
    else if (key == GLUT_KEY_UP)
    {
      win_pan_y -= .1*scale_factor;
      display_flag = 1;
      glutPostRedisplay();  
    } 
    else if (key == GLUT_KEY_DOWN)
    {
      win_pan_y += .1*scale_factor;
      display_flag = 1;
      glutPostRedisplay();  
    }
    else if (key == GLUT_KEY_PAGE_DOWN)
    {
      scale_factor *= 1.05;
      display_flag = 1;
      glutPostRedisplay();  
    }
    else if (key == GLUT_KEY_PAGE_UP)
    {
      scale_factor /= 1.05;
      display_flag = 1;
      glutPostRedisplay();  
    }
  }  
  
  if(scale_factor < .000001)
    scale_factor = .000001;
  
  
  Project();
}

// this routine gets called when a normal key is pressed
void key(unsigned char ch,int x,int y)
{
  if (ch == '=' || ch == '+')
  {
    scale_factor /= 1.05;
    display_flag = 1;
    glutPostRedisplay();  
  }
  else if (ch == '-' || ch == '_')
  {
    scale_factor *= 1.05;
    display_flag = 1;
    glutPostRedisplay();  
  }
  else if (ch == 'm' || ch == 'M')
  {
    display_state = MOVE;
    menu_flag = 1;
    glutPostRedisplay();  
  }
  else if (ch == 'g' || ch == 'G')
  {
    display_state = SETGOAL;
    menu_flag = 1;
    glutPostRedisplay();  
  }
  else if (ch == 'p' || ch == 'P')
  {
    display_state = SETPOSE;
    menu_flag = 1; 
    glutPostRedisplay(); 
  }
  else if (ch == 'r' || ch == 'R')
  {
    display_state = RELOADMAP;
    menu_flag = 1; 
    glutPostRedisplay(); 
  }
  /*
  else if (ch == 'c' || ch == 'C')
  {
    display_state = MOVEROBOT;
    menu_flag = 1; 
    glutPostRedisplay(); 
  }
  */
  else if (ch == '[' || ch == '{')
  {
    max_grids_on_side *= 2;
    if(max_grids_on_side > (float)max(costmap->height, costmap->width))
        max_grids_on_side = (float)max(costmap->height, costmap->width);
    display_flag = 1; 
    glutPostRedisplay(); 
  }
  else if (ch == ']' || ch == '}')
  {
    max_grids_on_side /= 2;
    if(max_grids_on_side < 4)
        max_grids_on_side = 4;
    display_flag = 1; 
    glutPostRedisplay(); 
  }  


  if(scale_factor < .000001)
    scale_factor = .000001;
  
  
  left_pressed = 0;
  right_pressed = 0;
  
  Project();
}

// this routine is called when a mouse button is pressed or released
void mouse(int button, int mouse_state, int x, int y)
{
  float mouse_x = 2*((float)win_aspect_ratio)*((float)x)/((float)(win_width -1)) - ((float)win_aspect_ratio);
  float mouse_y = 2*((float)(win_height-1-y))/((float)(win_height-1)) - 1;

  float mouse_x_world = mouse_x*scale_factor - win_pan_x;
  float mouse_y_world = mouse_y*scale_factor - win_pan_y;
  
  float menu_bottom = ((float)win_height/2 - (float)menu_pixels)/((float)win_height/2);
  float menu_height = 1 - menu_bottom;
  float pxls = menu_height/(float)menu_pixels;
  float menu_half_width = (float)win_width*pxls/2;
  
  mouseover_state = NONE; 
  if(mouse_y >= menu_bottom)
  {
    if(mouse_y >= menu_bottom + 5*pxls && mouse_y <= menu_bottom + 30*pxls) // possibly on button
    {
      if(mouse_x >= 5*pxls-menu_half_width && mouse_x <= 95*pxls-menu_half_width)
      {
        mouseover_state = MOVE;
        menu_flag = 1;
        glutPostRedisplay(); 
      }
      else if(mouse_x >= 105*pxls-menu_half_width && mouse_x <= 195*pxls-menu_half_width)      
      {
        mouseover_state = SETGOAL;
        menu_flag = 1;
        glutPostRedisplay(); 
      }
      else if(mouse_x >= 205*pxls-menu_half_width && mouse_x <= 295*pxls-menu_half_width)
      {
        mouseover_state = SETPOSE;
        menu_flag = 1;
        glutPostRedisplay(); 
      }
      else if(mouse_x >= 305*pxls-menu_half_width && mouse_x <= 395*pxls-menu_half_width)
      {
        mouseover_state = RELOADMAP;
        menu_flag = 1;
        glutPostRedisplay(); 
      }
      /*
      else if(mouse_x >= 405*pxls-menu_half_width && mouse_x <= 495*pxls-menu_half_width)
      {
        mouseover_state = MOVEROBOT;
        menu_flag = 1;
        glutPostRedisplay(); 
      }
      */
      else if(mouse_x >= 505*pxls-menu_half_width && mouse_x <= 545*pxls-menu_half_width)
      {
        mouseover_state = UPRES;
        menu_flag = 1;
        glutPostRedisplay(); 
      }
      else if(mouse_x >= 555*pxls-menu_half_width && mouse_x <= 595*pxls-menu_half_width)
      {
        mouseover_state = DOWNRES;
        menu_flag = 1;
        glutPostRedisplay(); 
      }
    }
    
    if(mousedown_state == NONE && mouseover_state != NONE && button == GLUT_LEFT_BUTTON && mouse_state == GLUT_DOWN && left_pressed == 0)
    {
      left_pressed = 1;    
      mousedown_state = mouseover_state;
    }
    
    if(mousedown_state != NONE && button == GLUT_LEFT_BUTTON && mouse_state == GLUT_UP)
    {
        if(mousedown_state == mouseover_state)
        {
          display_state = mousedown_state;
        }
        mousedown_state = NONE;   
        left_pressed = 0;
        menu_flag = 1;
        glutPostRedisplay(); 
    }
    return;
  }
  else if(mousedown_state != NONE && button == GLUT_LEFT_BUTTON && mouse_state == GLUT_UP)
  {
    mouseover_state = NONE;
    mousedown_state = NONE;
    left_pressed = 0;
    menu_flag = 1;
    glutPostRedisplay(); 
    return;
  }
  
  // if we get to here then the mouse is below the menu
  if(display_state == MOVE)
  {
    if(button == GLUT_LEFT_BUTTON && mouse_state == GLUT_DOWN && left_pressed == 0)
    { 
      mouse_pan_x_last = mouse_x;
      mouse_pan_y_last = mouse_y;
      left_pressed = 1;   
      display_flag = 1; 
      glutPostRedisplay(); 
    }
    else if (button == GLUT_RIGHT_BUTTON && mouse_state == GLUT_DOWN)
    {
      mouse_zoom_y_init = mouse_y;
      scale_factor_init = scale_factor;
      right_pressed = 1;
    }
  }
  else if(display_state == SETGOAL)
  {
    if(button == GLUT_LEFT_BUTTON && mouse_state == GLUT_DOWN && left_pressed == 0)
    {   
      new_location_x = mouse_x_world;
      new_location_y = mouse_y_world;
      new_heading_x = 0;
      new_heading_y = 0;
      left_pressed = 1; 
      display_flag = 1;
      glutPostRedisplay();  
    }
    else if(button == GLUT_LEFT_BUTTON && mouse_state == GLUT_UP)
    {
      new_heading_x = mouse_x_world - new_location_x;
      new_heading_y = mouse_y_world - new_location_y;
      //ROS_INFO_STREAM("\n New Goal Heading: " << new_heading_x << ", " << new_heading_y << "\n");
      
      move_base_msgs::MoveBaseActionGoal msg;
      
      msg.goal.target_pose.header.frame_id = "/map";
      //msg.frame_id = "/map";
      
      float map_rad = 2/(float)max(costmap->height, costmap->width); 
      float absolute_goal_x = (new_location_x + 1)/map_rad*costmap->resolution; // in meters
      float absolute_goal_y = (new_location_y + 1)/map_rad*costmap->resolution; // in meters
      
      msg.goal.target_pose.pose.position.x = absolute_goal_x+map_x_offset;
      msg.goal.target_pose.pose.position.y = absolute_goal_y+map_y_offset;
      msg.goal.target_pose.pose.position.z = 0;
      
      float theta = atan2(new_heading_y,new_heading_x);
      float r = sqrt(2-(2*cos(theta)));
      
      if(r == 0)
        msg.goal.target_pose.pose.orientation.w = 1;          
      else
        msg.goal.target_pose.pose.orientation.w = sin(theta)/r; 

      msg.goal.target_pose.pose.orientation.x = 0;
      msg.goal.target_pose.pose.orientation.y = 0;
      msg.goal.target_pose.pose.orientation.z = r/2; 
      
      goal_pub.publish(msg);    
      
      display_state = MOVE;
    }      
  }  
  else if(display_state == SETPOSE)
  {
    if(button == GLUT_LEFT_BUTTON && mouse_state == GLUT_DOWN && left_pressed == 0)
    {   
      new_location_x = mouse_x_world;
      new_location_y = mouse_y_world;
      new_heading_x = 0;
      new_heading_y = 0;
      left_pressed = 1;   
      display_flag = 1; 
      glutPostRedisplay(); 
    }
    else if(button == GLUT_LEFT_BUTTON && mouse_state == GLUT_UP)
    {
      new_heading_x = mouse_x_world - new_location_x;
      new_heading_y = mouse_y_world - new_location_y;
      
      geometry_msgs::PoseStamped msg;
      
      msg.header.frame_id = "/map";
      
      float map_rad = 2/(float)max(costmap->height, costmap->width); 
      float absolute_goal_x = (new_location_x + 1)/map_rad*costmap->resolution; // in meters
      float absolute_goal_y = (new_location_y + 1)/map_rad*costmap->resolution; // in meters
      
      msg.pose.position.x = absolute_goal_x+map_x_offset;
      msg.pose.position.y = absolute_goal_y+map_y_offset;
      msg.pose.position.z = 0;
      
      float theta = atan2(new_heading_y,new_heading_x);
      float r = sqrt(2-(2*cos(theta)));
      if( r == 0)          
        msg.pose.orientation.w = 1;
      else
        msg.pose.orientation.w = sin(theta)/r; 
  
      msg.pose.orientation.x = 0;
      msg.pose.orientation.y = 0;
      msg.pose.orientation.z = r/2; 
      
      new_pose_pub.publish(msg);    
      
      display_state = MOVE;
    }      
  }
  
  if(button == GLUT_LEFT_BUTTON && mouse_state == GLUT_UP)
  {
    left_pressed = 0;  
    display_flag = 1; 
    glutPostRedisplay(); 
  }
  
  if(button == GLUT_RIGHT_BUTTON && mouse_state == GLUT_UP)
  {
    right_pressed = 0; 
    display_flag = 1; 
    glutPostRedisplay(); 
  }
  
  Project();
}


// this is called when a mouse is moved while a button is down
void active_motion(int x,int y)
{
  if(right_pressed == 0 && left_pressed == 0)
    return;
 
  if(mousedown_state != NONE)
    return;
  
  float mouse_x = 2*((float)win_aspect_ratio)*((float)x)/((float)(win_width -1)) - ((float)win_aspect_ratio);
  float mouse_y = 2*((float)(win_height-1-y))/((float)(win_height-1)) - 1;

  float mouse_x_world = mouse_x*scale_factor - win_pan_x;
  float mouse_y_world = mouse_y*scale_factor - win_pan_y;
  
  if(display_state == MOVE)
  {
    if(left_pressed == 1)
    {
      win_pan_x += scale_factor*(mouse_x - mouse_pan_x_last);
      win_pan_y += scale_factor*(mouse_y - mouse_pan_y_last);
    
      mouse_pan_x_last = mouse_x;
      mouse_pan_y_last = mouse_y;
      
      display_flag = 1; 
      glutPostRedisplay(); 
    }
  
    if(right_pressed == 1)
    {
      scale_factor = scale_factor_init - (mouse_y - mouse_zoom_y_init); 
      
      display_flag = 1; 
      glutPostRedisplay(); 
    }
  }
  else if(display_state == SETGOAL)
  {
    if(left_pressed == 1)
    {
      new_heading_x = mouse_x_world - new_location_x;
      new_heading_y = mouse_y_world - new_location_y;
      
      display_flag = 1; 
      glutPostRedisplay(); 
    }      
  }  
  else if(display_state == SETPOSE)
  {
    if(left_pressed == 1)
    {
      new_heading_x = mouse_x_world - new_location_x;
      new_heading_y = mouse_y_world - new_location_y;
      
      display_flag = 1; 
      glutPostRedisplay(); 
    }      
  }
  
  if(scale_factor < .000001)
    scale_factor = .000001;
  
  Project();
}

// this is called when a mouse is moved while a button is not down
void motion(int x,int y)
{
  if(right_pressed == 1 || left_pressed == 1)
    return;
    
  //if(mousedown_state != NONE)
  //  return; // (this should never happen)
  
  float mouse_x = 2*((float)win_aspect_ratio)*((float)x)/((float)(win_width -1)) - ((float)win_aspect_ratio);
  float mouse_y = 2*((float)(win_height-1-y))/((float)(win_height-1)) - 1;

  float menu_bottom = ((float)win_height/2 - (float)menu_pixels)/((float)win_height/2);
  float menu_height = 1 - menu_bottom;
  float pxls = menu_height/(float)menu_pixels;
  float menu_half_width = (float)win_width*pxls/2;
  
  if(mouseover_state != NONE)
  {
    menu_flag = 1;
    glutPostRedisplay();
  }  


  mouseover_state = NONE; 
  if(mouse_y >= menu_bottom)
  {
    if(mouse_y >= menu_bottom + 5*pxls && mouse_y <= menu_bottom + 30*pxls) // possibly on button
    {
      if(mouse_x >= 5*pxls-menu_half_width && mouse_x <= 95*pxls-menu_half_width)
        mouseover_state = MOVE;
      else if(mouse_x >= 105*pxls-menu_half_width && mouse_x <= 195*pxls-menu_half_width)         
        mouseover_state = SETGOAL;
      else if(mouse_x >= 205*pxls-menu_half_width && mouse_x <= 295*pxls-menu_half_width)
        mouseover_state = SETPOSE;
      else if(mouse_x >= 305*pxls-menu_half_width && mouse_x <= 395*pxls-menu_half_width)
        mouseover_state = RELOADMAP;
      /*
      else if(mouse_x >= 405*pxls-menu_half_width && mouse_x <= 495*pxls-menu_half_width)
        mouseover_state = MOVEROBOT;
      */
      else if(mouse_x >= 505*pxls-menu_half_width && mouse_x <= 545*pxls-menu_half_width)
        mouseover_state = UPRES;
      else if(mouse_x >= 555*pxls-menu_half_width && mouse_x <= 595*pxls-menu_half_width)
        mouseover_state = DOWNRES;
    }
    menu_flag = 1;
    glutPostRedisplay();
  }
  
  Project();
}

// this routine is called when nothing else is happening
void idle()
{
  if(display_state == RELOADMAP)
  {
    win_pan_x = 0;
    win_pan_y = 0;
    scale_factor = 1;
    max_grids_on_side = 500;
          
    destroy_map(costmap);
    costmap = load_map();
    //costmap = load_blank_map(1000, 1000, .01);
    
    if(costmap != NULL)
    {
        display_state = MOVE;
        //ROS_INFO("STATE: MOVE\n");
    }
    else
    {
        ROS_INFO("Failed to get map\n");
    }
  }
  else if(display_state == UPRES)
  {  
    max_grids_on_side *= 2;
    if(max_grids_on_side > (float)max(costmap->height, costmap->width))
        max_grids_on_side = (float)max(costmap->height, costmap->width);
    display_flag = 1; 
    glutPostRedisplay(); 
    
    display_state = MOVE;
  }
  else if(display_state == DOWNRES)
  {  
    max_grids_on_side /= 2;
    if(max_grids_on_side < 4)
        max_grids_on_side = 4;
    display_flag = 1; 
    glutPostRedisplay(); 
    
    display_state = MOVE;
  }
  
  ros::spinOnce();
    
  if(display_flag == 1 || menu_flag == 1)
      glutPostRedisplay();         
  //else
  //    loop_rate.sleep();
}

// this gets called when glut exits
void cleanup()
{

  particle_cloud_sub.shutdown();
  robot_footprint_sub.shutdown();
  global_path_sub.shutdown();
  local_path_sub.shutdown();
  inflated_obs_local_sub.shutdown();
  obstacles_local_sub.shutdown();

  goal_pub.shutdown();
  new_pose_pub.shutdown();
  
  destroy_map(costmap);
  destroy_robot(robot);
  destroy_point_list(global_path);
  destroy_point_list(local_path);
  destroy_grid_list(inflated_obs_local);
  destroy_grid_list(obstacles_local);
  
  ROS_INFO("\nExit... \n");  
  
  exit(0);
}

int main(int argc, char *argv[])
{   
  // init ROS stuff  
  ros::init(argc, argv, "visualization_cu");  
  ros::NodeHandle nh;
  ROS_INFO("Welcome to CU visualization");
  
  // global variable init stuff
  robot =  make_robot(0, 0, 0, RED);
  
  // set up ROS topic subscriber callbacks
  particle_cloud_sub = nh.subscribe("/particlecloud", 2, particle_cloud_callback);
  robot_footprint_sub = nh.subscribe("/local_costmap/robot_footprint", 2, robot_footprint_callback);
  global_path_sub = nh.subscribe("/NavfnROS/plan", 2, global_plan_callback);
  local_path_sub = nh.subscribe("/TrajectoryPlannerROS/local_plan", 2, local_plan_callback);
  inflated_obs_local_sub = nh.subscribe("/local_costmap/inflated_obstacles", 2, inflated_obs_local_callback);
  obstacles_local_sub = nh.subscribe("/local_costmap/obstacles", 2, obstacles_local_callback);
  
  // set up ROS topic publishers
  goal_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 2);
  new_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base/current_position", 2);
  
  //  Initialize GLUT
  glutInit(&argc,argv);
  
  //  Request double buffered, true color window with Z buffering at 600x600
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize(display_size_init,display_size_init);
  glutCreateWindow("Visualization_CU");
  
  //  Set GLUT callbacks
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutSpecialFunc(special);
  glutKeyboardFunc(key);

  glutPassiveMotionFunc(motion);
  glutMouseFunc(mouse);
  glutMotionFunc(active_motion);
  glutIdleFunc(idle);
  atexit(cleanup);
  
  glutPostRedisplay();
  
  //  Pass control to GLUT so it can interact with the user
  glutMainLoop();
  
  return 0;
}

    
