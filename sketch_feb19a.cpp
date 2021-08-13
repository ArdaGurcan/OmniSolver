//using UnityEngine;

// class MazeSolving : MonoBehaviour
// {
/// <summary>
/// First explore the whole maze using trémaux algorithm.
/// When the goal is found find shortest path to start using floodfill.
/// At every step, look at the surrounding obstacles, write them to the grid and find path again using floodfill.
/// When reached start, do the same to reach the goal.
/// -Arda
/// Yorumlara bakarak daha iyi anlayabilirsin ne oldugunu
/// </summary>

#include <AFMotor.h>
#include <NewPing.h>


AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

const int maxDistance = 5 * US_ROUNDTRIP_CM;

NewPing up_sensor(A2, A2, maxDistance);
NewPing right_sensor(A4, A4, maxDistance);
NewPing down_sensor(A5, A5, maxDistance);
NewPing left_sensor(A3, A3, maxDistance);

int motor1Speed = 100;
int motor2Speed = 100;


const int arrayLength = 6;
int distances[4][6];
int averageDistances[] = {0, 0, 0, 0};
bool wall[] = {0, 0, 0, 0};
//for debugging
int currentIndex = 1;
// Vector2 gotPosition = new Vector2(22, 0);
int robotPosition[] = {22, 0};
// int[,] grid = new int[23, 23]; // stores each wall, available slot, and wall corners
char grid[23][23];
int state; // which step is currently being executed
bool backtracking;

int goalPositions[2][2];

int solveGrid[23][23];

// int returnGrid[23][23];

int smallest = 145;

int destination[] = {22, 0};
bool moving = false;

void setup()
{
  for (int i = 0; i < sizeof(goalPositions); i++)
  {
    for (int j = 0; j < sizeof(goalPositions[i]); j++)
    {
      goalPositions[i][j] = 0;
    }
  }

  for (int i = 0; i < sizeof(solveGrid); i++)
  {
    for (int j = 0; j < sizeof(solveGrid[i]); j++)
    {
      solveGrid[i][j] = 0;
    //   returnGrid[i][j] = 0;
      grid[i][j] = 0;
    }
  }

  motor1.setSpeed(motor1Speed);
  motor2.setSpeed(motor2Speed);
  motor3.setSpeed(200-motor1Speed);
  motor4.setSpeed(200-motor2Speed);
  // initialize grid with wall corners as -1's
  for (int i = 0; i < 23; i++)
  {
    for (int j = 0; j < 23; j++)
    {
      if (i * j % 2 != 0)
      {
        grid[i][j] = -1;
      }
    }
  }
  // InvokeRepeating("Solve", 0, 0.1f);
}

void loop() // switch between each step according to state variable
{
  if (!moving)
  {
    for (int i = 0; i < 4; i++)
    {
      unsigned int uS = 0;
      switch (i)
      {
      case 0:
        uS = up_sensor.ping();
        break;

      case 1:
        uS = right_sensor.ping();
        break;

      case 2:
        uS = down_sensor.ping();
        break;

      case 3:
        uS = left_sensor.ping();
        break;
      }

      int cm = uS / US_ROUNDTRIP_CM;

      if (cm != 0)
      {
        for (int j = 0; j < arrayLength - 1; j++)
        {
          distances[i][j] = distances[i][j + 1];
        }

        distances[i][arrayLength - 1] = cm;
      }
    }

    for (int i = 0; i < 4; i++)
    {
      float sum = 0;
      for (int j = 0; j < arrayLength - 1; j++)
      {
        sum += distances[i][j];
      }
      averageDistances[i] = sum / arrayLength;
      if (averageDistances[i] < 10)
      {
        wall[i] = true;
      }
      else
      {
        wall[i] = false;
      }
    }

    switch (state)
    {
    case 0:
      Step0();
      break;
    case 1:
      Step1();
      break;
    case 2:
      Step2();
      break;

    }
  }

  /// COLLISION
}

void Step0() // explore the whole maze until goal is found
{
  //write current index to the robot position in grid
  grid[round(robotPosition[0])][round(robotPosition[1])] = currentIndex;

  //write any seen walls to grid
  if (wall[0] && IsInGrid(robotPosition[1], robotPosition[0] - 1))
  {
    grid[round(robotPosition[0]) - 1][round(robotPosition[1])] = -1;
  }
  if (wall[1] && IsInGrid(robotPosition[1] + 1, robotPosition[0]))
  {
    // //print("right sensor");
    grid[round(robotPosition[0])][round(robotPosition[1]) + 1] = -1;
  }
  if (wall[2] && IsInGrid(robotPosition[1], robotPosition[0] + 1))
  {
    grid[round(robotPosition[0]) + 1][round(robotPosition[1])] = -1;
  }
  if (wall[3] && IsInGrid(robotPosition[1] - 1, robotPosition[0]))
  {
    ////print("left sensor");
    grid[round(robotPosition[0])][round(robotPosition[1]) - 1] = -1;
  }

  //move robot
  if (!backtracking)
  {
    //if there isn't a wall and it hasn't been numbered
    if (!wall[0] && !IsNumbered(0))
    {
      ////print("moving forward");
      Move(0);
      currentIndex++;
    }
    else if (!wall[1] && !IsNumbered(1))
    {
      //print("moving right");
      Move(1);
      currentIndex++;
    }
    else if (!wall[3] && !IsNumbered(3))
    {
      //print("moving left");
      Move(3);
      currentIndex++;
    }
    else if (!wall[2] && !IsNumbered(2))
    {
      //print("moving back");
      Move(2);
      currentIndex++;
    }
    else
    {
      //print("stuck");
      backtracking = true;
    }
  }
  else //if stuck
  {
    //backtrack

    if (!wall[0] && !IsNumbered(0)) // if the slot is empty
    {
      //print("moving forward1");
      Move(0);
      backtracking = false;
    }
    else if (!wall[1] && !IsNumbered(1))
    {
      //print("moving right1");
      Move(1);
      backtracking = false;
      currentIndex++;
    }
    else if (!wall[3] && !IsNumbered(3))
    {
      //print("moving left1");
      Move(3);
      backtracking = false;
      currentIndex++;
    }
    else if (!wall[2] && !IsNumbered(2))
    {
      //print("moving back1");
      Move(2);
      backtracking = false;
      currentIndex++;
    }
    else if (!wall[0] && Number(0) == currentIndex - 1) // if still stuck but backtracking check if 1 lower than current index
    {
      //print("moving forward2");
      Move(0);
      currentIndex--;
    }
    else if (!wall[1] && Number(1) == currentIndex - 1)
    {
      //print("moving right2");
      Move(1);
      currentIndex--;
    }
    else if (!wall[3] && Number(3) == currentIndex - 1)
    {
      //print("moving left2");
      Move(3);
      currentIndex--;
    }
    else if (!wall[2] && Number(2) == currentIndex - 1)
    {
      //print("moving back2");
      Move(2);
      currentIndex--;
    }
  }

  ////print grid with robot position
  //////printGrid(grid);
}

void Step1()
{
  //go back to start while discovering shortest path
  if (wall[0] && IsInGrid(robotPosition[1], robotPosition[0] - 1))
  {
    //print("up sensor");
    grid[round(robotPosition[0]) - 1][round(robotPosition[1])] = -1;
  }
  if (wall[1] && IsInGrid(robotPosition[1] + 1, robotPosition[0]))
  {
    //print("right sensor");
    grid[round(robotPosition[0])][round(robotPosition[1]) + 1] = -1;
  }
  if (wall[2] && IsInGrid(robotPosition[1], robotPosition[0] + 1))
  {
    //print("down sensor");
    grid[round(robotPosition[0]) + 1][round(robotPosition[1])] = -1;
  }
  if (wall[3] && IsInGrid(robotPosition[1] - 1, robotPosition[0]))
  {
    //print("left sensor");
    grid[round(robotPosition[0])][round(robotPosition[1]) - 1] = -1;
  }
  FillSolveGrid();
  smallest = 145;

  if (!wall[0] && IsInGrid(round(robotPosition[0]) - 2, round(robotPosition[1])) && solveGrid[round(robotPosition[0]) - 2][round(robotPosition[1])] < smallest)
  {
    //print("Up available");
    smallest = solveGrid[round(robotPosition[0]) - 2][round(robotPosition[1])];
  }
  if (!wall[1] && IsInGrid(round(robotPosition[0]), round(robotPosition[1]) + 2) && solveGrid[round(robotPosition[0])][round(robotPosition[1]) + 2] < smallest)
  {
    //print("Right available");
    smallest = solveGrid[round(robotPosition[0])][round(robotPosition[1]) + 2];
  }
  if (!wall[3] && IsInGrid(round(robotPosition[0]), round(robotPosition[1]) - 2) && solveGrid[round(robotPosition[0])][round(robotPosition[1]) - 2] < smallest)
  {
    //print("Left available");
    smallest = solveGrid[round(robotPosition[0])][round(robotPosition[1]) - 2];
  }
  if (!wall[2] && IsInGrid(round(robotPosition[0]) + 2, round(robotPosition[1])) && solveGrid[round(robotPosition[0]) + 2][round(robotPosition[1])] < smallest)
  {
    //print("Down available");
    smallest = solveGrid[round(robotPosition[0] + 2)][round(robotPosition[1])];
  }

  if (!wall[0] && IsInGrid(round(robotPosition[0]) - 2, round(robotPosition[1])) && solveGrid[round(robotPosition[0]) - 2][round(robotPosition[1])] == smallest)
  {
    //print("Moving up");
    Move(0);
  }
  else if (!wall[1] && IsInGrid(round(robotPosition[0]), round(robotPosition[1]) + 2) && solveGrid[round(robotPosition[0])][round(robotPosition[1]) + 2] == smallest)
  {
    //print("Moving right");
    Move(1);
  }
  else if (!wall[3] && IsInGrid(round(robotPosition[0]), round(robotPosition[1]) - 2) && solveGrid[round(robotPosition[0])][round(robotPosition[1]) - 2] == smallest)
  {
    //print("Moving left");
    Move(3);
  }
  else if (!wall[2] && IsInGrid(round(robotPosition[0]) + 2, round(robotPosition[1])) && solveGrid[round(robotPosition[0]) + 2][round(robotPosition[1])] == smallest)
  {
    //print("Moving down");
    Move(2);
  }

  if (robotPosition[0] == 22 && robotPosition[1] == 0)
  {
    destination[0] = goalPositions[0][0];
    destination[1] = goalPositions[0][1];
    FillSolveGrid();
    state = 2;
  }
  //state = 2; //temporary
}

void Step2()
{
  //go back to goal while discovering shortest path
  if (wall[0] && IsInGrid(robotPosition[1], robotPosition[0] - 1))
  {
    //print("up sensor");
    grid[round(robotPosition[0]) - 1][round(robotPosition[1])] = -1;
  }
  if (wall[1] && IsInGrid(robotPosition[1] + 1, robotPosition[0]))
  {
    //print("right sensor");
    grid[round(robotPosition[0])][round(robotPosition[1]) + 1] = -1;
  }
  if (wall[2] && IsInGrid(robotPosition[1], robotPosition[0] + 1))
  {
    //print("down sensor");
    grid[round(robotPosition[0]) + 1][round(robotPosition[1])] = -1;
  }
  if (wall[3] && IsInGrid(robotPosition[1] - 1, robotPosition[0]))
  {
    //print("left sensor");
    grid[round(robotPosition[0])][round(robotPosition[1]) - 1] = -1;
  }
  FillSolveGrid();
  smallest = 145;
  if (!wall[0] && IsInGrid(round(robotPosition[0]) - 2, round(robotPosition[1])) && solveGrid[round(robotPosition[0]) - 2][round(robotPosition[1])] < smallest)
  {
    //print("Up available");
    smallest = solveGrid[round(robotPosition[0]) - 2][round(robotPosition[1])];
  }
  if (!wall[1] && IsInGrid(round(robotPosition[0]), round(robotPosition[1]) + 2) && solveGrid[round(robotPosition[0])][round(robotPosition[1]) + 2] < smallest)
  {
    //print("Right available");
    smallest = solveGrid[round(robotPosition[0])][round(robotPosition[1]) + 2];
  }
  if (!wall[3] && IsInGrid(round(robotPosition[0]), round(robotPosition[1]) - 2) && solveGrid[round(robotPosition[0])][round(robotPosition[1]) - 2] < smallest)
  {
    //print("Left available");
    smallest = solveGrid[round(robotPosition[0])][round(robotPosition[1]) - 2];
  }
  if (!wall[2] && IsInGrid(round(robotPosition[0]) + 2, round(robotPosition[1])) && solveGrid[round(robotPosition[0]) + 2][round(robotPosition[1])] < smallest)
  {
    //print("Down available");
    smallest = solveGrid[round(robotPosition[0] + 2)][round(robotPosition[1])];
  }

  if (!wall[0] && IsInGrid(round(robotPosition[0]) - 2, round(robotPosition[1])) && solveGrid[round(robotPosition[0]) - 2][round(robotPosition[1])] == smallest)
  {
    //print("Moving up");
    Move(0);
  }
  else if (!wall[1] && IsInGrid(round(robotPosition[0]), round(robotPosition[1]) + 2) && solveGrid[round(robotPosition[0])][round(robotPosition[1]) + 2] == smallest)
  {
    //print("Moving right");
    Move(1);
  }
  else if (!wall[3] && IsInGrid(round(robotPosition[0]), round(robotPosition[1]) - 2) && solveGrid[round(robotPosition[0])][round(robotPosition[1]) - 2] == smallest)
  {
    //print("Moving left");
    Move(3);
  }
  else if (!wall[2] && IsInGrid(round(robotPosition[0]) + 2, round(robotPosition[1])) && solveGrid[round(robotPosition[0]) + 2][round(robotPosition[1])] == smallest)
  {
    //print("Moving down");
    Move(2);
  }

  if (robotPosition[0] == goalPositions[0][0] && robotPosition[1] == goalPositions[0][1])
  {
    destination[0] = 22;
    destination[1] = 0;
    FillSolveGrid();
    state = 1;
  }
}

bool IsNumbered(int direction) // check if the available square in given direction has been assigned a number before
{
  switch (direction)
  {
  case 0:
    if (grid[round(robotPosition[0]) - 2][round(robotPosition[1])] == 0)
    {
      return false;
    }
    break;
  case 1:
    if (grid[round(robotPosition[0])][round(robotPosition[1]) + 2] == 0)
    {
      return false;
    }
    break;
  case 2:
    if (grid[round(robotPosition[0]) + 2][round(robotPosition[1])] == 0)
    {
      return false;
    }
    break;
  case 3:
    if (grid[round(robotPosition[0])][round(robotPosition[1]) - 2] == 0)
    {
      return false;
    }
    break;
  }
  return true;
}

int Number(int direction) // get the number in the current square
{
  switch (direction)
  {
  case 0:
    return grid[round(robotPosition[0]) - 2][round(robotPosition[1])];
  case 1:
    return grid[round(robotPosition[0])][round(robotPosition[1]) + 2];
  case 2:
    return grid[round(robotPosition[0]) + 2][round(robotPosition[1])];
  case 3:
    return grid[round(robotPosition[0])][round(robotPosition[1]) - 2];
  }
  return -1;
}

void Move(int direction) // move robot and update grid
{
  switch (direction)
  {
  case 0:
    //    transform.position += transform.forward; // 20 cm git
    goUp();
    robotPosition[0] = robotPosition[0] - 2;
    robotPosition[1] = robotPosition[1];

    break;
  case 1:
    //    transform.position += transform.right;
    goRight();

    //robotPosition = {robotPosition[0], robotPosition[1] + 2};
    robotPosition[0] = robotPosition[0];
    robotPosition[1] = robotPosition[1] + 2;
    break;
  case 2:
    //    transform.position -= transform.forward;
    goDown();

    // robotPosition = {robotPosition[0] + 2, robotPosition[1]};
    robotPosition[0] = robotPosition[0] + 2;
    robotPosition[1] = robotPosition[1];

    break;
  case 3:
    //    transform.position -= transform.right;
    goLeft();
    // robotPosition = {robotPosition[0], robotPosition[1] - 2};
    robotPosition[0] = robotPosition[0];
    robotPosition[1] = robotPosition[1] - 2;

    break;
  }
}

bool IsInGrid(int x, int y) // check if position is in the bounds of the grid
{
  if (x >= 0 && x < 23 && y >= 0 && y < 23)
  {
    return true;
  }
  return false;
}

void FillSolveGrid() // fill another grid called "solveGrid" according  to original grid
{
  //  solveGrid = new int[23, 23];
  int solveGrid[23][23];

  for (int i = 0; i < 23; i++)
  {
    for (int j = 0; j < 23; j++)
    {
      if (grid[i][j] == -1)
      {
        solveGrid[i][j] = -1;
      }
      else if (destination[0] != i || destination[1] != j)
      {
        solveGrid[i][j] = 145;
      }
      else if (destination[0] == i && destination[1] == j)
      {
        solveGrid[i][j] = 0;
      }
    }
  }

  for (int i = 0; i < 145; i++)
  {
    for (int y = 0; y < 23; y += 2)
    {
      for (int x = 0; x < 23; x += 2)
      {
        if (solveGrid[y][x] == i)
        {
          if (y - 2 > -1 && y - 2 < 23 && solveGrid[y - 1][x] != -1 && solveGrid[y - 2][x] == 145)
          {
            solveGrid[y - 2][x] = i + 1;
          }
          if (y + 2 > -1 && y + 2 < 23 && solveGrid[y + 1][x] != -1 && solveGrid[y + 2][x] == 145)
          {
            solveGrid[y + 2][x] = i + 1;
          }
          if (x - 2 > -1 && x - 2 < 23 && solveGrid[y][x - 1] != -1 && solveGrid[y][x - 2] == 145)
          {
            solveGrid[y][x - 2] = i + 1;
          }
          if (x + 2 > -1 && x + 2 < 23 && solveGrid[y][x + 1] != -1 && solveGrid[y][x + 2] == 145)
          {
            solveGrid[y][x + 2] = i + 1;
          }
        }
      }
    }
  }
}

void FindExit()
{
  int pos = 0;
  for (int i = 1; i <= 18; i += 2)
  {
    for (int j = 1; j <= 18; j += 2)
    {
      int wallSum =
        8 +
        grid[i + 1][j] +
        grid[i + 3][j] +
        grid[i][j + 1] +
        grid[i][j + 3] +
        grid[i + 4][j + 1] +
        grid[i + 4][j + 3] +
        grid[i + 1][j + 4] +
        grid[i + 3][j + 4];

      int emptySum =
        grid[i + 2][j + 1] +
        grid[i + 2][j + 3] +
        grid[i + 1][j + 2] +
        grid[i + 3][j + 2];

      if (wallSum == 7 && emptySum == 0)
      {
        //        goalPositions[pos] = {i + 1,
        //          j + 1};
        goalPositions[pos][0] = i + 1;
        goalPositions[pos][1] = j + 1;
        pos++;
      }
    }
  }

  for (int i = 0; i < 2; i++)
  {
    if (goalPositions[i][0] == 0 && goalPositions[i][1] == 0)
    {
      //      goalPositions[i] = {goalPositions[0][0], goalPositions[0][1]};
      goalPositions[i][0] = goalPositions[0][0];
      goalPositions[i][1] = goalPositions[0][1];
    }
  }
}

void up()
{
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void down()
{
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void right()
{
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}

void left()
{
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}

void stop()
{
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void adjustUp()
{
  // adjust the speed of the motors so the robot is tilting up
  motor1Speed++;

  motor2Speed--;
}

void adjustDown()
{
  // adjust the speed of the motors so the robot is tilting down
  motor2Speed++;
  motor1Speed--;

}

void adjustLeft()
{
  // adjust the speed of the motors so the robot is tilting left

  motor1Speed--;
  motor2Speed--;
}

void adjustRight()
{
  // adjust the speed of the motors so the robot is tilting right
  motor1Speed++;
  motor2Speed++;

}

void goUp()
{
  int distance = averageDistances[0];
  up();
  moving = true;
  while (averageDistances[0] > distance - 20)
  {
    if (averageDistances[3] > averageDistances[1])
    {
      adjustRight();

    }
    else
    {
      adjustLeft();
    }
    motor1.setSpeed(motor1Speed);
  motor2.setSpeed(motor2Speed);
  motor3.setSpeed(200-motor1Speed);
  motor4.setSpeed(200-motor2Speed);
  }
  moving = false;
  stop();
}

void goRight()
{
  int distance = averageDistances[1];
  right();
  moving = true;
  while (averageDistances[1] > distance - 20)
  {
    if (averageDistances[0] > averageDistances[2])
    {
      adjustDown();
    }
    else
    {
      adjustUp();
    }
    motor1.setSpeed(motor1Speed);
  motor2.setSpeed(motor2Speed);
  motor3.setSpeed(200-motor1Speed);
  motor4.setSpeed(200-motor2Speed);
  }
  moving = false;
  stop();
}

void goDown()
{
  int distance = averageDistances[2];
  down();
  moving = true;
  while (averageDistances[2] > distance - 20)
  {
    if (averageDistances[1] > averageDistances[3])
    {
      adjustRight();
    }
    else
    {
      adjustLeft();
    }
    motor1.setSpeed(motor1Speed);
  motor2.setSpeed(motor2Speed);
  motor3.setSpeed(200-motor1Speed);
  motor4.setSpeed(200-motor2Speed);
  }
  moving = false;
  stop();
}

void goLeft()
{
  int distance = averageDistances[3];
  left();
  moving = true;
  while (averageDistances[3] > distance - 20)
  {
    if (averageDistances[2] > averageDistances[0])
    {
      adjustUp();
    }
    else
    {
      adjustDown();
    }
    motor1.setSpeed(motor1Speed);
  motor2.setSpeed(motor2Speed);
  motor3.setSpeed(200-motor1Speed);
  motor4.setSpeed(200-motor2Speed);
  }
  moving = false;
  stop();
}
