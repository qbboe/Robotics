////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////// ROBOTICS ////////////// UNIVERSITA' DEGLI STUDI DI PAVIA /////////////// A.A. 2015/2016 //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// NAVIGATION ALGORITHM BUG 1 //////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// BOERCI EMMANUELE ///// N. 423082 //////////////////////////////////////////////////////////////////////////////////////////////////
//////// RIGONI SIMONE //////// N. 428116 //////////////////////////////////////////////////////////////////////////////////////////////////
//////// ZANCHETTA MATTIA ///// N. 427857 //////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>

#include <math.h>

#include <unistd.h>

#include "morse.h"

#include "socket.h"

# define PI 3.14159265358979323846

//////////////////////////////////////////// USER PARAMETER ////////////////////////////////////////////////////////////////////////////////

# define WALLDISTANCE 1.0  //Defines the distance that the robot keeps from the obstacle in boundary following

//////////////////////////////////////////// MAIN FUNCTION ////////////////////////////////////////////////////////////////////////////////

int main() {

// Variables declaration

	double v = 0;

	double vmax=0.1;

	double w = 0;

	double x, y;

	int lap=0;	int tril=0;				// tril = 0: MTG; 1: first lap; 2: reach nearest point; 3: GOAL

	double nearest_distance=100, nearest_x, nearest_y, start_x, start_y; // Values to compute for the Bug1 algorithm

	double x_A, y_A, x_B, y_B, x_C, y_C, x_P, y_P, x_G, y_G, d_A, d_B, d_C, k, h, j, xnum, xden, m;  // Values for the trilateration functions which compute the goal point

	double target[3]={0.0, 0.0, PI};  //Set initial values for the goal (these are going to be changed once the trilateration is completed)

	int motion=0, state=0, fl=0, hole=0;   //Variables to go through all the states in the switch functions of the algorithm

	double min_distance, min_yaw, start_yaw, heading, yaw_offset, min_x, min_y, distance;  //Variables used to make the turns

// Call for the Morse structures related to the sensors of the robot

	Pose pose;

	proxMeas proxmeas;

	irMeas IR1, IR2, IR3, IR4;    



// Set the address and the port for the socket communication

	char address[] = "127.0.0.1";

	int serverPort = 4000;

// Set the name of the robot

	char parent[] = "robot";

// Open the communication (here all the components use the same port)

	int sock;

	sock = client_doSock(address, serverPort);

	if ( sock == -1) {

        printf("Errore 1\n");

        return 1;

	}

	FILE * f = fdopen(sock, "r+");

// Get the pose measurement

	int flag = getPose(f, parent, "pose", &pose);

	if ( flag < 0 ) {

	printf("Errore 3\n");

	fclose(f);

	return 2;

	}

// Get the proximity measurement

	flag = getProx(f, parent, "prox", &proxmeas);

	if ( flag < 0 ) {

	printf("Errore 5\n");

	fclose(f);

	return 3;

	}

// Initialize the values for the trilateration function which will return the goal coordinates

	x_A=pose.x;				// The coordinates of 3 points are needed to perform the trilateration, along with their distance from the goal. These are hereafter named A,B,C

	y_A=pose.y;

	d_A=proxmeas.objects->dist;

	tril=1;					// Once the first point has been saved the robot can start moving to get the other 2 points, 

	x_B=x_A;

	y_B=y_A;	

	x_C=x_A;

	y_C=y_A;



//////////////////////////////////////////// ROBOT MOTION LOOP ////////////////////////////////////////////////////////////////////////////////

  while (lap<3) {							// The algorithm will exit the loop when the goal has been reached (the condition being lap==3)

// Set the robot linear and angular speed

		int flag = setSpeed(f, parent, "motion", v, w);

		if ( flag < 0 ) {

		    printf("Errore 2\n");

		    fclose(f);

		    return 2;

		}

// Get the pose measurement

		flag = getPose(f, parent, "pose", &pose);

		if ( flag < 0 ) {

		    printf("Errore 3\n");

		    fclose(f);

		    return 2;

		}

// Get the proximity measurement

		flag = getProx(f, parent, "prox", &proxmeas);

		if ( flag < 0 ) {

		    printf("Errore 5\n");

		    fclose(f);

		    return 3;

		}

// Get the infrared 1 measurement

		flag = getIR(f, parent, "IR1", &IR1);

		if ( flag < 0 ) {

		    printf("Errore 6\n");

		    fclose(f);

		    return 4;

		}

// Get the infrared 2 measurement

		flag = getIR(f, parent, "IR2", &IR2);

		if ( flag < 0 ) {

		    printf("Errore 7\n");

		    fclose(f);

		    return 4;

		}

// Get the infrared 3 measurement

		flag = getIR(f, parent, "IR3", &IR3);

		if ( flag < 0 ) {

		    printf("Errore 8\n");

		    fclose(f);

		    return 4;

		}

// Get the infrared 4 measurement

		flag = getIR(f, parent, "IR4", &IR4);

		if ( flag < 0 ) {

		    printf("Errore 9\n");

		    fclose(f);

		    return 4;

		}


// The following small function sets a yaw offset in order to have the yaw angle going from 0 to 2*PI, rather than -PI to PI radians

	if(pose.yaw<0) 	{ yaw_offset=(pose.yaw)+2*PI; }									// If the yaw is negative adds 2*PI, otherwise keep the value, so the minimum yaw is 0 and the maximum yaw becomes 2*PI radians

	else 			{ yaw_offset=pose.yaw; }



// Compute distances for both coordinates (terms dx and dy) and yaw angle (th) to the goal

	double dx = -pose.x + target[0];

	double dy = -pose.y + target[1];

	double th = atan2(dy, dx) - pose.yaw;

	distance=sqrt(pow(dx,2) + pow(dy,2));											// Compute the nearest distance		

	if (lap==2 && abs(pose.x-nearest_x)<0.3 && abs(pose.y-nearest_y)<0.3){motion=0;vmax=0.2;}	// During the second lap the robot goes back to the point where it had found the minimum distance, and switches back to Motion to Goal

	if (lap==1 && abs(pose.x-start_x)<0.5 && abs(pose.y-start_y)<0.5 && distance-nearest_distance>1) {lap=2;}	// At the end of the first lap, the variable "lap" is updated so now the robot will go back to the nearest point to the goal

	if (distance < nearest_distance) {												// The distance to the goal is constantly checked, until a minimum value is found, according to the Bug1 rules

		nearest_x=pose.x;

		nearest_y=pose.y;

		nearest_distance = distance; }



/////// Goal coordinates computation by means of trilateration

	if(tril<3) {									// The trilateration ends when the variable "tril" is equal to 3

		x_P=pose.x;									// The coordinates of the first point have been already saved, the 2 remaining points are now computed

		y_P=pose.y;									// The point P is used as a parameter to go through the computation

		switch(tril) {

			case 1:

			k=abs(x_A-x_P)+abs(y_A-y_P);			// The robot travels a certain distance, and when the value of k exceeds 1 the current position and distance from the goal are saved, regarding point B

			if(k>1){				

				x_B=pose.x;			

				y_B=pose.y;

				d_B=proxmeas.objects->dist;			

				tril=2; }							// Point B has been found, the trilateration function goes on to compute C

			break;

			case 2:

			k=abs(x_A-x_P)+abs(y_A-y_P);			// The computation is similar to the one done for point B (case tril=1), the robot travels a distance and finds the point

			h=abs(x_B-x_P)+abs(y_B-y_P);

			m=(y_A-y_B)/(x_A-x_B+0.001);			// These 3 lines prevent C from being on the same line as A and B, which would void the results of the trilateration

			y_G=m*(pose.x-x_B)+y_B;

			j=abs(y_G-pose.y);

			if(k>1 && h>1 && j>0.2){				// Once the condition is reached point C is saved as it was previously done for A and B

				x_C=pose.x;			

				y_C=pose.y;

				d_C=proxmeas.objects->dist;			

				////// The trilateration is performed using point A, B and C, the following lines give the two coordinates of the goal

				xnum=((pow(d_A,2)-pow(d_B,2))*(y_A-y_C)-(pow(d_A,2)-pow(d_C,2))*(y_A-y_B)-(pow(x_A,2)-pow(x_B,2)+pow(y_A,2)-pow(y_B,2))*(y_A-y_C)+(pow(x_A,2)-pow(x_C,2)+pow(y_A,2)-pow(y_C,2))*(y_A-y_B));

				xden=(2*(x_A-x_C)*(y_A-y_B)-2*(x_A-x_B)*(y_A-y_C));

				x=xnum/xden;

				y=(-2*x*(x_A-x_C)-pow(d_A,2)+pow(d_C,2)+pow(x_A,2)-pow(x_C,2)+pow(y_A,2)-pow(y_C,2))/(2*(y_A-y_C));

				// The two coordinates have been found and are saved as the coordinates of the target, the value of "tril" is set to 3 thus closing this part of the loop and the speed is set to the nominal value

				target[0]=x;

				target[1]=y;

				tril=3;

				vmax=1;

				}

			break;
		}	
	}

///////////////// The goal has been computed, the algorithm can start the motion switching between "motion to goal" and "boundary following"

///////////////// Motion To Goal								

	switch(motion)			// Motion = 0  ->  Motion to Goal,     Motion = 1  ->  Boundary Following

	{

	case 0:   				// Motion to Goal

		if(tril<3) {		// While trilateration is being performed, keep the linear speed to a low value to improve computation accuracy

		v = vmax;

		} else {			// After trilateration is completed set the speed go decrease with the distance to the goal thus stopping the robot when the goal has been reached

		v = vmax*distance;

		}

		w = 0.5 * th;		// Rotate towards the computed goal

		if (proxmeas.objects->dist<0.3) {v=0; w=0; lap=3; usleep(10000);}				// The Goal has been reached and the condition is set "lap=3"

		else if(*IR1.dist<0.6||*IR2.dist<0.6||*IR3.dist<0.6) {							// If there is an obstacle go to boundary following					

			state=0;

			motion++;			

			}		

		break;

	case 1:

///////////////// Boundary Following								

		switch(state)

		{

			case 0: 										// In this case the robot saves the data regarding the point where it encountered the obstacle

				v=0.1;										//stop the robot and save yaw and front distance, then start rotating

				start_yaw = yaw_offset;

				min_distance = *IR1.dist;

				heading=0;

				fl=0;

				w=-1.0;

				state++;

				break;

			case 1: 											// state=1: The robot begins to rotate until it reaches a certain angle wrt the obstacle and goes to state=2

				if(*IR2.dist<0.6&&*IR2.dist>0.5&&*IR3.dist==2.0) {

				state++;

				}

				break;

			case 2: 											

				if (lap==0){start_x=pose.x; start_y=pose.y; lap=1;}		// If the lap of the obstacle is the first one save the starting point of encounter

				if(*IR2.dist<2.0) {							// This small subroutine defines the normal motion along the obstacle keeping a fixed distance from it which can be altered by the user

				v=vmax;

				w=2*(*IR2.dist-WALLDISTANCE);

				}

				if(*IR2.dist==2.0&&*IR1.dist==2.0) {		// If the sensor suddenly reads 2.0 there must be a hole in the wall or a left corner 

				min_yaw=yaw_offset;							// Save current position and yaw for subroutine computations

				min_y=pose.y;

				min_x=pose.x;

				fl=0;										// Sets a flag and the switch parameter "hole" for the subroutine that deals with a hole in the wall

				hole=0;

				state=3;									// Jumps to the corresponding subroutine

				}	

				if(*IR1.dist<0.8)  {						// Found a right hand corner, the program saves the current yaw and jumps to the corresponding subroutine

				state=4;

				}	

				if(*IR1.dist==2.0&&*IR3.dist<1.0) {			// Found a gap in the wall which could be too narrow to go through, the robot goes backwards a bit and then treats it like a corner

				state=5;

				min_y=pose.y;

				min_x=pose.x;

				}

				break;

			case 3:												// Subroutine for dealing with a hole in the wall, it is divided in 4 sections: the robot goes straight a bit over its lenghts, turns 90 degrees left, goes forward again over its length and turns left again to realign with the obstacle and proceed with Boundary Following

				switch(hole)									// 3 cases

				{

				case 0:

					if(fl==0) {									// case 0: keep moving forward for a certain distance, then rotate 90 degrees left

					w=0;

					distance=fabs(sqrt(pow(pose.x-min_x,2) + pow(pose.y-min_y,2)));		// Computes travelled distance from the saved point

					if(distance>1.2){ 							// Travels a certain distance

					fl=1;

						}

					}

					if(fl==1) {						

					heading = min_yaw + PI/2 - yaw_offset;			// Computes the target angle for the turn

					if(heading>2*PI) {heading=heading-2*PI;}		// These two instructions are mandatory to keep the computed yaw angle between 0 and 2*PI thus not compromising the logic

					if(heading<-2*PI) {heading=heading+2*PI;}

					w=2.0*heading;									// Turns 90 degrees wrt to the yaw angle the robot had when the hole was found

					if(fabs(w)<0.2) {					// When close enough to the target angle proceed (this speeds up the cornering manoeuvre significantly)

						if(*IR1.dist<1.0)  {			// If a corner is found, the program jumps to the corresponding subroutine

						state=4;

						break;

						}								// If there is no corner then there is a hole so the robot needs to do another left hand turn to realign on the other side of the obstacle

						min_y=pose.y;

						min_x=pose.x;

						w=0;

						hole++;

						}

					}

					break;

				case 1:										// Hole = Case 1: travel forwards for a small distance and turn 90 degrees left to realign with the obstacle 

					if(fl==1) {

					distance=fabs(sqrt(pow(pose.x-min_x,2) + pow(pose.y-min_y,2)));  // Computes the travelled distance

					if(*IR1.dist<1.0)  {			// Found a corner, the program jumps to the corresponding subroutine

						state=4;

						break;

						}

					if(distance>0.6){ 				// If no corner is found, when a certain distance has been travelled the robot makes the second left hand turn

						min_yaw=yaw_offset;

						fl=2;

						}

					}

					if(fl==2) {

					heading = min_yaw + PI/2 - yaw_offset; 			// Computes the target angle for the second turn (a total of PI radians wrt to yaw angle the beginning of this switch construct)

					if(heading>2*PI) {heading=heading-2*PI;}

					if(heading<-2*PI) {heading=heading+2*PI;}

					w=2.0*heading;									// Turns again 90 degrees wrt to the yaw angle the robot had when it made the first turn

					if(fabs(w)<0.1) {

						if(*IR1.dist<1.0)  {			// If a corner is found, the program jumps to the corresponding subroutine

						state=4;

						break;

						}								// The robot has self aligned along the obstacle, it can now resume Boundary Following as soon as it can see the obstacle on his left side sensor

						w=0;

						hole++;

						}

					}

					break;

				case 2:										// Hole = Case 2: travel forwards and as soon as sensor 2 reads < 2, which means the obstacle is there and it can go back to standard boundary following (state=2)

					if(*IR2.dist<2) {

					state=2;

					}

					break;

				}

				break;										// Exit the subroutine to deal with a hole in the wall

			case 4:												// Subroutine when a corner is found going along the obstacle, turn 90 degrees right and go back to standard boundary following (state=2)

				v=0.1;										// The robot starts rotating right	

				w=-0.5;

				if(*IR1.dist==2.0&&*IR2.dist==2.0&&*IR3.dist==2.0&&*IR4.dist<0.8) {  //This sudden condition can be met only if there is a very small wall, and in this case the robot turns around it

				v=1.0;										// The subroutine is the same one that the robot uses when there is a hole in the wall

				min_yaw=yaw_offset;							// Save current position and yaw for subroutine computations

				min_y=pose.y;

				min_x=pose.x;

				fl=0;										// Sets a flag and the switch parameters for the subroutine (state=3)

				hole=0;

				state=3;	

				}

				if((*IR2.dist<0.6&&*IR2.dist>0.5&&*IR4.dist<2.0)||(*IR2.dist<0.6&&*IR2.dist>0.5&&*IR1.dist==2.0&&*IR4.dist==2.0)) {			// As soon as it reaches the condition the robot is aligned with the next side of the wall and it goes back to std boundary following (state=2)

				state=2;

				}

				break;

			case 5:								// In this case there is a small gap in the wall, so the readings are not identical to those obtained in normal conditions. The robot has to go backwards a bit, and then make a right hand turn as it had found a corner

				w=0;

				v=-1;

				distance=fabs(sqrt(pow(pose.x-min_x,2) + pow(pose.y-min_y,2)));		// Computes travelled distance from the saved point

				if(distance>0.2) 	{				//  After travelling in reverse for a bit turns right as if there was a corner in front instead of a gap
			
				min_yaw=yaw_offset;

				state=4;

				}

				break;

			case 20:											// This state can be used for tests, and to check whether there are infinite loops or not in computations, or checking where a certain subroutine stops

				v=0;

				w=0;

				break;

		}

		break;

	} 						// Here the "motion" switch construct ends

////////////////// DATA CHECK
// Some useful data is hereafter saved on a file, to check the performances of the different subroutines and the conditions of the switch constructs and flags

		FILE * fp;

				fp = fopen ("target.txt", "w+");

				fprintf(fp, "target[0] = %lf, target[1] = %lf;\n", target[0], target[1]);

				fprintf(fp, "motion = %d, state = %d, flag = %d\n", motion, state, fl);

				fprintf(fp, "finalyaw = %lf, startyaw = %lf, min_distance = %lf,  min_yaw = %lf, th = %lf\n", pose.yaw, start_yaw, min_distance, min_yaw, th);

				fprintf(fp, "heading = %lf, yaw_offset = %lf, IR2 = %lf,  hole = %d, distance = %lf\n", heading, yaw_offset, *IR2.dist, hole, distance);

				fprintf(fp, "IR1 = %lf, IR2 = %lf, IR3 = %lf,  IR4 = %lf, v = %lf\n", *IR1.dist, *IR2.dist, *IR3.dist, *IR4.dist, v);

				fprintf(fp,"\nlap = %d \nnearest distance = %lf, nearestx= %lf, nearesty= %lf \nposex = %lf, posey = %lf",lap, nearest_distance, nearest_x, nearest_y, pose.x, pose.y);			

				fprintf(fp, "\nv = %lf, w = %lf, x = %lf, y = %lf", v, w, x, y);

		fclose(fp);			

  }

// When the goal is reached the following code opens a file and writes a statement thus enabling the user to check whether the algorithm has been successful

		FILE * fg;

				fg = fopen ("target.txt", "w+"); 		

				fprintf(fg, "GOAL REACHED!");

		fclose(fg);	

// Close the socket

	fclose(f);

	return 0;

}

// The main function of the algorithm ends here



