using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace Common_Namespace
{
    public class SimpleOperations
    {
        public static double RadiusE(double Latitude, double Altitude) //R_1
        {
            return SimpleData.A / Math.Pow(1 - SimpleData.Ex_Squared * Math.Sin(Latitude) * Math.Sin(Latitude), 0.5) + Altitude;
        }

        public static double RadiusN(double Latitude, double Altitude) //R_2
        {
            return SimpleData.A * (1 - SimpleData.Ex_Squared) / Math.Pow(1 - SimpleData.Ex_Squared * Math.Sin(Latitude) * Math.Sin(Latitude), 1.5) + Altitude;
        }

        public static double GilmertGravityForce(double Latitude, double Altitude)
        {
            return 9.780318 * (1 + 0.005302 * Math.Sin(Latitude) * Math.Sin(Latitude) - 0.000007 * Math.Sin(2 * Latitude) * Math.Sin(2 * Latitude)) - 0.00014 - 2 * Altitude * 0.000001543;
        }
        public static double NormalGravity_g0(double Latitude, double Altitude)
        {
            return 9.780318 * (1 + 0.005302 * Math.Sin(Latitude) * Math.Sin(Latitude) - 0.0000059 * Math.Sin(2 * Latitude) * Math.Sin(2 * Latitude)) ;
        }

        public static double AbsoluteVectorValue(double[] Vect)
        {
            return Math.Sqrt(Vect[0] * Vect[0] + Vect[1] * Vect[1] + Vect[2] * Vect[2]);
        }

        public static double SkalyarProduct(double[] Vect1, double[] Vect2)
        {
            return (Vect1[0] * Vect2[0] + Vect1[1] * Vect2[1] + Vect1[2] * Vect2[2]);
        }

        public static double[] NullingOfArray(double[] Vect)
        {
            for (int i = 0; i < Vect.Length; i++)
                Vect[i] = 0.0;

            return Vect;
        }
        public static double[] NullingOfNotDiagMatrixElements(double[] Vect, int matrixDim)
        {
            for (int i = 0; i < matrixDim; i++)
            {
                for (int j = 0; j < matrixDim; j++)
                {
                    if (i != j)
                        Vect[i * matrixDim + j] = 0.0;
                }
            }

            return Vect;
        }
        public static double[] NullingOfBottomMatrix(double[] Vect, int matrixDim)
        {
            for (int i = 0; i < matrixDim; i++)
            {
                for (int j = 0; j < i; j++)
                {
                    Vect[i * matrixDim + j] = 0.0;
                }
            }

            return Vect;
        }

        //---------------------------------Задается модель поведения движения-------------------------------------------

        public static double CalculateDistanceBtwDots(double Lat1, double Lon1, double Alt1, double Lat2, double Lon2, double Alt2)
        {
            double lat_dif_true = (Lat2 - Lat1) * SimpleOperations.RadiusN(Lat2, Alt2);
            double long_dif_true = (Lon2 - Lon1) * SimpleOperations.RadiusE(Lat2, Alt2) * Math.Cos(Lat2);

            return Math.Sqrt(lat_dif_true * lat_dif_true + long_dif_true * long_dif_true);
        }
        public static double CalculateHeadingByTwoDots(double Lat1, double Lon1, double Alt1, double Lat2, double Lon2, double Alt2)
        {
            double lat_dif_true = (Lat2 - Lat1) * SimpleOperations.RadiusN(Lat2, Alt2);
            double long_dif_true = (Lon2 - Lon1) * SimpleOperations.RadiusE(Lat2, Alt2) * Math.Cos(Lat2);

            return Math.Atan2(long_dif_true, lat_dif_true);
        }

        //---------------------------------------------------------------------------------------------------------------

        public static double[] U_x0(double Latitude)
        {
            double[] ResultVect = new double[3];
            ResultVect[0] = 0.0;
            ResultVect[1] = SimpleData.U * Math.Cos(Latitude);
            ResultVect[2] = SimpleData.U * Math.Sin(Latitude);
            return ResultVect;
        }


        //-----------------------------------------------------------------Матрицы----------------------------------------------------------
        public static Matrix A_sx0(double[] Angles)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = Math.Cos(Angles[0]) * Math.Cos(Angles[1]) + Math.Sin(Angles[0]) * Math.Sin(Angles[2]) * Math.Sin(Angles[1]);
            MatrixResult[0, 1] = -Math.Sin(Angles[0]) * Math.Cos(Angles[1]) + Math.Cos(Angles[0]) * Math.Sin(Angles[2]) * Math.Sin(Angles[1]);
            MatrixResult[0, 2] = -Math.Cos(Angles[2]) * Math.Sin(Angles[1]);
            MatrixResult[1, 0] = Math.Sin(Angles[0]) * Math.Cos(Angles[2]);
            MatrixResult[1, 1] = Math.Cos(Angles[0]) * Math.Cos(Angles[2]);
            MatrixResult[1, 2] = Math.Sin(Angles[2]);
            MatrixResult[2, 0] = Math.Cos(Angles[0]) * Math.Sin(Angles[1]) - Math.Sin(Angles[0]) * Math.Sin(Angles[2]) * Math.Cos(Angles[1]);
            MatrixResult[2, 1] = -Math.Sin(Angles[0]) * Math.Sin(Angles[1]) - Math.Cos(Angles[0]) * Math.Sin(Angles[2]) * Math.Cos(Angles[1]);
            MatrixResult[2, 2] = Math.Cos(Angles[2]) * Math.Cos(Angles[1]);
            return MatrixResult;
        }
        public static Matrix A_sx0(double Heading, double Roll, double Pitch)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = Math.Cos(Heading) * Math.Cos(Roll) + Math.Sin(Heading) * Math.Sin(Pitch) * Math.Sin(Roll);
            MatrixResult[0, 1] = -Math.Sin(Heading) * Math.Cos(Roll) + Math.Cos(Heading) * Math.Sin(Pitch) * Math.Sin(Roll);
            MatrixResult[0, 2] = -Math.Cos(Pitch) * Math.Sin(Roll);
            MatrixResult[1, 0] = Math.Sin(Heading) * Math.Cos(Pitch);
            MatrixResult[1, 1] = Math.Cos(Heading) * Math.Cos(Pitch);
            MatrixResult[1, 2] = Math.Sin(Pitch);
            MatrixResult[2, 0] = Math.Cos(Heading) * Math.Sin(Roll) - Math.Sin(Heading) * Math.Sin(Pitch) * Math.Cos(Roll);
            MatrixResult[2, 1] = -Math.Sin(Heading) * Math.Sin(Roll) - Math.Cos(Heading) * Math.Sin(Pitch) * Math.Cos(Roll);
            MatrixResult[2, 2] = Math.Cos(Pitch) * Math.Cos(Roll);
            return MatrixResult;
        }
        public static Matrix A_sx0(SINS_State SINSState)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = Math.Cos(SINSState.Heading) * Math.Cos(SINSState.Roll) + Math.Sin(SINSState.Heading) * Math.Sin(SINSState.Pitch) * Math.Sin(SINSState.Roll);
            MatrixResult[0, 1] = -Math.Sin(SINSState.Heading) * Math.Cos(SINSState.Roll) + Math.Cos(SINSState.Heading) * Math.Sin(SINSState.Pitch) * Math.Sin(SINSState.Roll);
            MatrixResult[0, 2] = -Math.Cos(SINSState.Pitch) * Math.Sin(SINSState.Roll);
            MatrixResult[1, 0] = Math.Sin(SINSState.Heading) * Math.Cos(SINSState.Pitch);
            MatrixResult[1, 1] = Math.Cos(SINSState.Heading) * Math.Cos(SINSState.Pitch);
            MatrixResult[1, 2] = Math.Sin(SINSState.Pitch);
            MatrixResult[2, 0] = Math.Cos(SINSState.Heading) * Math.Sin(SINSState.Roll) - Math.Sin(SINSState.Heading) * Math.Sin(SINSState.Pitch) * Math.Cos(SINSState.Roll);
            MatrixResult[2, 1] = -Math.Sin(SINSState.Heading) * Math.Sin(SINSState.Roll) - Math.Cos(SINSState.Heading) * Math.Sin(SINSState.Pitch) * Math.Cos(SINSState.Roll);
            MatrixResult[2, 2] = Math.Cos(SINSState.Pitch) * Math.Cos(SINSState.Roll);
            return MatrixResult;
        }
        public static Matrix A_xs(SINS_State SINSState)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = Math.Cos(SINSState.Roll);
            MatrixResult[0, 1] = 0.0;
            MatrixResult[0, 2] = Math.Sin(SINSState.Roll);
            MatrixResult[1, 0] = Math.Sin(SINSState.Pitch) * Math.Sin(SINSState.Roll);
            MatrixResult[1, 1] = Math.Cos(SINSState.Pitch);
            MatrixResult[1, 2] = -Math.Sin(SINSState.Pitch) * Math.Cos(SINSState.Roll);
            MatrixResult[2, 0] = -Math.Cos(SINSState.Pitch) * Math.Sin(SINSState.Roll);
            MatrixResult[2, 1] = Math.Sin(SINSState.Pitch);
            MatrixResult[2, 2] = Math.Cos(SINSState.Pitch) * Math.Cos(SINSState.Roll);
            return MatrixResult;
        }
        public static Matrix A_xs(double Heading, double Roll, double Pitch)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = Math.Cos(Roll);
            MatrixResult[0, 1] = 0.0;
            MatrixResult[0, 2] = Math.Sin(Roll);
            MatrixResult[1, 0] = Math.Sin(Pitch) * Math.Sin(Roll);
            MatrixResult[1, 1] = Math.Cos(Pitch);
            MatrixResult[1, 2] = -Math.Sin(Pitch) * Math.Cos(Roll);
            MatrixResult[2, 0] = -Math.Cos(Pitch) * Math.Sin(Roll);
            MatrixResult[2, 1] = Math.Sin(Pitch);
            MatrixResult[2, 2] = Math.Cos(Pitch) * Math.Cos(Roll);
            return MatrixResult;
        }



        public static Matrix ArrayToMatrix(double[] array)
        {
            Matrix MatrixResult = new Matrix(Convert.ToInt32(Math.Sqrt(array.Length)), Convert.ToInt32(Math.Sqrt(array.Length)));
            for (int i = 0; i < Convert.ToInt32(Math.Sqrt(array.Length)); i++)
            {
                for (int j = 0; j < Convert.ToInt32(Math.Sqrt(array.Length)); j++)
                {
                    MatrixResult[i, j] = array[i * Convert.ToInt32(Math.Sqrt(array.Length)) + j];
                }
            }
            return MatrixResult;
        }
        public static Matrix ArrayToDiagonalMatrix(double[] array)
        {
            Matrix MatrixResult = new Matrix(array.Length, array.Length);
            for (int i = 0; i < array.Length; i++)
            {
                MatrixResult[i, i] = array[i];
            }
            return MatrixResult;
        }


        public static Matrix ArrayToMatrix(double[] array, int Rows, int Cols)
        {
            Matrix MatrixResult = new Matrix(Rows, Cols);

            for (int i = 0; i < Rows; i++)
            {
                for (int j = 0; j < Cols; j++)
                {
                    MatrixResult[i, j] = array[i * Cols + j];
                }
            }
            return MatrixResult;
        }


        public static double[] MatrixToArray(Matrix matrix)
        {
            double[] array = new double[matrix.Cols*matrix.Rows];
            for (int i = 0; i < matrix.Rows; i++)
            {
                for (int j = 0; j < matrix.Cols; j++)
                {
                    array[i * matrix.Cols + j] = matrix[i, j];
                }
            }
            return array;
        }

        public static Matrix A_x0n(double Latitude, double Longitude)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = -Math.Sin(Longitude);
            MatrixResult[0, 1] = Math.Cos(Longitude);
            MatrixResult[0, 2] = 0.0;
            MatrixResult[1, 0] = -Math.Cos(Longitude) * Math.Sin(Latitude);
            MatrixResult[1, 1] = -Math.Sin(Longitude) * Math.Sin(Latitude);
            MatrixResult[1, 2] = Math.Cos(Latitude);
            MatrixResult[2, 0] = Math.Cos(Longitude) * Math.Cos(Latitude);
            MatrixResult[2, 1] = Math.Sin(Longitude) * Math.Cos(Latitude);
            MatrixResult[2, 2] = Math.Sin(Latitude);
            return MatrixResult;
        }
        
        public static Matrix A_ne(double CurrentTime, double StartLongitude)
        {
            Matrix MatrixResult = new Matrix(3, 3);
            MatrixResult[0, 0] = Math.Cos(SimpleData.U * CurrentTime);// + StartLongitude);
            MatrixResult[0, 1] = Math.Sin(SimpleData.U * CurrentTime);// + StartLongitude);
            MatrixResult[0, 2] = 0.0;
            MatrixResult[1, 0] = -Math.Sin(SimpleData.U * CurrentTime);// + StartLongitude);
            MatrixResult[1, 1] = Math.Cos(SimpleData.U * CurrentTime);// + StartLongitude);
            MatrixResult[1, 2] = 0.0;
            MatrixResult[2, 0] = 0.0;
            MatrixResult[2, 1] = 0.0;
            MatrixResult[2, 2] = 1.0;
            return MatrixResult;
        }

        public static void CopyArray(double[] p, double[] p_2)
        {
            for (int ii = 0; ii < Math.Min(p.Length, p_2.Length); ii++)
                p[ii] = p_2[ii];
        }
        public static void CopyArray(double[,] p, int i, double[] p_2)
        {
            for (int ii = 0; ii < p_2.Length; ii++)
                p[i,ii] = p_2[ii];
        }
        public static void CopyArray(double[] p, int i, double[,] p_2)
        {
            for (int ii = 0; ii < p.Length; ii++)
                p[ii] = p_2[i,ii];
        }
        public static void CopyMatrix(Matrix p, Matrix p_2)
        {
            for (int ii = 0; ii < p.Rows; ii++)
                for (int jj = 0; jj < p.Cols; jj++)
                    p[ii,jj] = p_2[ii,jj];
        }




        public static Matrix MultiplyUpperMatrix(Matrix mLeft, Matrix mRight)
        {
            Matrix mOut = new Matrix(mLeft.Rows, mRight.Cols);

            for (int i = 0; i < mOut.Rows; i++)
                for (int j = i; j < mOut.Cols; j++)
                    for (int k = j; k < mLeft.Cols; k++)
                        mOut[i, j] += mLeft[i, k] * mRight[k, j];
            return mOut;
        }
        public static Matrix MultiplyRightUpperMatrix(Matrix mLeft, Matrix mRight)
        {
            Matrix mOut = new Matrix(mLeft.Rows, mRight.Cols);

            for (int i = 0; i < mOut.Rows; i++)
                for (int j = 0; j < mOut.Cols; j++)
                    for (int k = 0; k <= j; k++)
                        mOut[i, j] += mLeft[i, k] * mRight[k, j];
            return mOut;
        }

        public static Matrix MultiplyVectorForMatrix(double[] mLeft, double[] mRight)
        {
            Matrix mOut = new Matrix(mLeft.Length, mRight.Length);

            for (int i = 0; i < mOut.Rows; i++)
                for (int j = 0; j < mOut.Cols; j++)
                    mOut[i, j] = mLeft[i] * mRight[j];

            return mOut;
        }




        public static void MakeMatrixFromVector(Matrix Matrix_out, double[] Vector_in, int dim)
        {
            for (int i = 0; i < dim; i++)
            {
                for (int j = 0; j < dim; j++)
                {
                    Matrix_out[i, j] = Vector_in[i * dim + j];
                }
            }
        }


        public static void PrintMatrixToFile(double[] Matrix, int dim_str, int dim_colls)
        {
            StreamWriter PrintMatrix = new StreamWriter(SimpleData.PathOutputString + "\\PrintMatrix.txt");
            for (int j = 0; j < dim_str; j++)
            {
                string str_odo = null;
                for (int i = 0; i < dim_colls; i++)
                {
                    str_odo = str_odo + " " + Matrix[j * dim_colls + i].ToString();
                }

                PrintMatrix.WriteLine(str_odo);
            }
            PrintMatrix.Close();
        }
        public static void PrintMatrixToFile(double[] Matrix, int dim_str, int dim_colls, string nameFile)
        {
            StreamWriter PrintMatrix = new StreamWriter(SimpleData.PathOutputString + "\\PrintMatrix_" + nameFile + ".txt");
            for (int j = 0; j < dim_str; j++)
            {
                string str_odo = null;
                for (int i = 0; i < dim_colls; i++)
                {
                    str_odo = str_odo + " " + Matrix[j * dim_colls + i].ToString();
                }

                PrintMatrix.WriteLine(str_odo);
            }
            PrintMatrix.Close();
        }

        public static void PrintMatrixToFile_TinyToZero(double[] Matrix, int dim_str, int dim_colls, string nameFile)
        {
            StreamWriter PrintMatrix = new StreamWriter(SimpleData.PathOutputString + "\\PrintMatrix_" + nameFile + ".txt");
            for (int j = 0; j < dim_str; j++)
            {
                string str_odo = null;
                for (int i = 0; i < dim_colls; i++)
                {
                    double tmp = Matrix[j * dim_colls + i];
                    if (Math.Abs(Matrix[j * dim_colls + i]) < 1E-4)
                        tmp = 0.0;
                    str_odo = str_odo + " " + tmp.ToString();
                }

                PrintMatrix.WriteLine(str_odo);
            }
            PrintMatrix.Close();
        }

        public static void PrintVectorToFile(double[] Vector, int dim)
        {
            StreamWriter PrintVector = new StreamWriter(SimpleData.PathOutputString + "\\PrintVector.txt");
            for (int j = 0; j < dim; j++)
            {
                string str_odo = null;

                str_odo = str_odo + " " + Vector[j].ToString();

                PrintVector.WriteLine(str_odo);
            }
            PrintVector.Close();
        }
        
       
    }
}
