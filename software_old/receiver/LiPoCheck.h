// VoltageCheck.h

/*
Copyright 2015 - 2017 Andreas Chaitidis Andreas.Chaitidis@gmail.com

This program is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.

*/


#define MINVOLRANGE 3.5
#define MAXVOLRANGE 4.2
#define MAXNUMBERCELLS  22

//const float lipoDc[2][8] = { { 0.0, 3.670,3.740,3.790,3.820,3.900,4.030,4.200 } , {0.000, 0.100,0.200,0.300,0.400,0.600,0.800,1.000 } };
const float lipoDc[2][11] = { { 3.2, 3.680,3.740,3.780,3.810,3.850,3.870,3.960,4.02,4.100,4.200 } ,{ 0.000, 0.100,0.200,0.300,0.400,0.500,0.600,0.700,0.800,0.900,1.000 } };
int CountCells(float voltage);

int CapCheckPerc(float voltage, int cells);


