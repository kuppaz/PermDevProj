using System;
using System.Text;
using System.Linq;
using System.IO;
using System.Collections;
using System.Runtime.Serialization.Formatters.Binary;

namespace Common_Namespace
{
    /*
* Операторы:
* унарный:
* -
* бинарные:
* с матрицами:
*  +, -, *
* с матрицой и числом:
* matrix / k, k * matrix, matrix * k
*/

    /// <summary>
    /// класс, описывающий матрицы, содержащие элементы типа double
    /// </summary>
    [Serializable()]
    public class Matrix : IEnumerable
    {
        double[,] m;

        /// <summary>
        /// создаёт матрицу размерами iRows x iCols
        /// </summary>
        /// <param name="iRows">количество строк</param>
        /// <param name="iCols">количество столбцов</param>
        public Matrix(int iRows, int iCols)
        {
            if (iRows <= 0) throw new MatrixException("В матрице нет строк");
            if (iCols <= 0) throw new MatrixException("В матрице нет столбцов");
            m = new double[iRows, iCols];
        }
        /// <summary>
        /// создаёт столбец размерами iRows x 1
        /// </summary>
        /// <param name="iRows">количество строк</param>
        public Matrix(int iRows)
        {
            if (iRows <= 0) throw new MatrixException("В векторе нет строк");
            m = new double[iRows, 1];
        }
        /// <summary>
        /// количество строк
        /// </summary>
        public int Rows
        {
            get { return m.GetLength(0); }
        }
        /// <summary>
        /// количество столбцов
        /// </summary>
        public int Cols
        {
            get { return m.GetLength(1); }
        }
        /// <summary>
        /// индексатор матрицы
        /// индексы начинаются с 0
        /// </summary>
        public double this[int iRow, int iCol]
        {
            get { return m[iRow, iCol]; }
            set { m[iRow, iCol] = value; }
        }
        /// <summary>
        /// индексатор столбца
        /// индекс начинается с 0
        /// </summary>
        public double this[int iRow]
        {
            get { return m[iRow, 0]; }
            set { m[iRow, 0] = value; }
        }

        /// <summary>
        /// представляет как строку
        /// </summary>		
        public override string ToString()
        {
            StringBuilder sOut = new StringBuilder();
            for (int i = 0; i < this.Rows; i++)
            {
                for (int j = 0; j < this.Cols; j++)
                    sOut.Append(this[i, j].ToString("E") + " ");
                sOut.Append("\n");
            }
            return sOut.ToString();
        }
        /// <summary>
        /// заплняет случайным образом
        /// </summary>
        public void Randomize()
        {
            Random rnd = new Random();
            for (int i = 0; i < this.Rows; i++)
                for (int j = 0; j < this.Cols; j++)
                    this[i, j] = rnd.NextDouble();
        }
        /// <summary>
        /// добавляет данные из файла в матрицу
        /// </summary>
        /// <param name="file">путь</param>
        public void LoadFromFile(string file)
        {
            try
            {
                string[] lines = System.IO.File.ReadAllLines(file);
                string[] line;
                if (lines.Length != this.Rows) throw new Exception("Некорректная размерность");
                for (int i = 0; i < lines.Length; i++)
                {
                    line = lines[i].Split(' ');
                    if (line.Length != this.Cols) throw new Exception("Некорректная размерность");
                    for (int j = 0; j < line.Length; j++)
                        this[i, j] = Convert.ToDouble(line[j]);
                }
            }
            catch (IOException e)
            {
                throw new MatrixException("Ошибка доступа к файлу матрицы", e);
            }
            catch (FormatException e)
            {
                throw new MatrixException("Файл имеет неверный формат", e);
            }
            catch (NullReferenceException e)
            {
                throw new MatrixException("Файл имеет неверный формат", e);
            }
            catch (Exception e)
            {
                throw new MatrixException("Неизвестная ошибка", e);
            }
        }
        /// <summary>
        /// сохраняет матрицу в файл
        /// </summary>
        /// <param name="file">путь к файлу на диске</param>
        public void SaveToFile(string file)
        {
            try
            {
                using (StreamWriter outfile = new StreamWriter(file))
                {
                    outfile.Write(this);
                }
            }
            catch (Exception e)
            {
                throw new MatrixException("Ошибка сохранения каталога в файл", e);
            }
        }
        /// <summary>
        /// Сохраняет матрицу в бинарный файл
        /// </summary>
        /// <param name="file">путь к файлу</param>
        public void SaveToBinaryFile(string file)
        {
            try
            {
                using (FileStream str = File.Create(file))
                {
                    BinaryFormatter bf = new BinaryFormatter();
                    bf.Serialize(str, this);
                }
            }
            catch (IOException e)
            {
                throw new MatrixException("Ошибка доступа к файлу матрицы", e);
            }
            catch (FormatException e)
            {
                throw new MatrixException("Файл имеет неверный формат", e);
            }
            catch (NullReferenceException e)
            {
                throw new MatrixException("Файл имеет неверный формат", e);
            }
            catch (Exception e)
            {
                throw new MatrixException("Неизвестная ошибка", e);
            }
        }
        /// <summary>
        /// Загружает матрицу из бинарного файла
        /// </summary>
        /// <param name="file">путь к файлу</param>
        public void LoadFromBinaryFile(string file)
        {
            try
            {
                Matrix loadedMatrix;
                using (FileStream str = File.OpenRead(file))
                {
                    BinaryFormatter bf = new BinaryFormatter();
                    loadedMatrix = (Matrix)bf.Deserialize(str);
                }
                this.m = loadedMatrix.m;
            }
            catch (Exception e)
            {
                throw new MatrixException("Ошибка доступа к файлу каталога", e);
            }
        }
        /// <summary>
        /// LU разложение
        /// </summary>
        public void GetLU(out Matrix L, out Matrix U)
        {
            //этим же методом можно решать СЛАУ
            if (this.Rows != this.Cols) throw new MatrixException("Матрица должна быть квадратной");
            L = new Matrix(this.Rows, this.Cols);
            U = new Matrix(this.Rows, this.Cols);

            for (int i = 0; i < this.Rows; i++)					//сперто с википедии=)
            {
                for (int j = 0; j < this.Rows; j++)
                {
                    U[0, i] = this[0, i];
                    L[i, 0] = this[i, 0] / U[0, 0];
                    double sum = 0;
                    for (int k = 0; k < i; k++)
                    {
                        sum += L[i, k] * U[k, j];
                    }
                    U[i, j] = this[i, j] - sum;
                    if (i > j)
                    {
                        L[j, i] = 0;
                    }
                    else
                    {
                        sum = 0;
                        for (int k = 0; k < i; k++)
                        {
                            sum += L[j, k] * U[k, i];
                        }
                        L[j, i] = (this[j, i] - sum) / U[i, i];
                    }
                }
            }
        }
        /// <summary>
        /// создает единичную матрицу размерами n x n
        /// </summary>
        /// <param name="n">размер</param>
        /// <returns>единичную матрицу</returns>
        public static Matrix UnitMatrix(int n)
        {
            if (n <= 0) throw new MatrixException("Некорректная размерность");
            Matrix mOut = new Matrix(n, n);
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    mOut[i, j] = (i == j) ? 1 : 0;
            return mOut;
        }
        /// <summary>
        /// возвращает такую же матрицу
        /// </summary>
        public Matrix Clone()
        {
            Matrix clone = new Matrix(this.Rows, this.Cols);
            for (int i = 0; i < this.Rows; i++)
                for (int j = 0; j < this.Cols; j++)
                    clone[i, j] = this[i, j];
            return clone;
        }
        /// <summary>
        /// рассширенная матрица. "склеивает" 2 матрицы
        /// </summary>
        /// <param name="mLeft">левая матрица n x m</param>
        /// <param name="mRight">правая матрица n x k</param>
        /// <returns>рассширенную матрицу размерами n x (m+k)</returns>
        private static Matrix ExtendMatrix(Matrix mLeft, Matrix mRight)
        {
            if (mLeft.Rows != mRight.Rows) throw new MatrixException("Разное количество строк");
            Matrix mOut = new Matrix(mLeft.Rows, mLeft.Cols + mRight.Cols);
            for (int i = 0; i < mLeft.Rows; i++)
            {
                for (int j = 0; j < mLeft.Cols; j++)
                    mOut[i, j] = mLeft[i, j];
                for (int j = 0; j < mRight.Cols; j++)
                    mOut[i, j + mLeft.Cols] = mRight[i, j];
            }
            return mOut;
        }
        /// <summary>
        /// транспонирует матрицу
        /// </summary>
        public Matrix Transpose()
        {
            Matrix mOut = new Matrix(this.Cols, this.Rows);
            for (int i = 0; i < mOut.Rows; i++)
                for (int j = 0; j < mOut.Cols; j++)
                    mOut[i, j] = this[j, i];
            return mOut;
        }
        /// <summary>
        /// минор матрицы
        /// </summary>
        /// <param name="iRow">строка</param>
        /// <param name="iCol">столбец</param>
        /// <returns>матрицу без iRow и iCol</returns>
        public Matrix Minor(int iRow, int iCol)
        {
            try
            {
                Matrix minor = new Matrix(this.Rows - 1, this.Cols - 1);
                int m = 0, n = 0;
                for (int i = 0; i < this.Rows; i++)
                {
                    if (i == iRow) continue;
                    n = 0;
                    for (int j = 0; j < this.Cols; j++)
                    {
                        if (j == iCol) continue;
                        minor[m, n] = this[i, j];
                        n++;
                    }
                    m++;
                }
                return minor;
            }
            catch (IndexOutOfRangeException e)
            {
                throw new MatrixException("Индексы вышли за пределы размеров матрицы", e);
            }
            catch (Exception e)
            {
                throw new MatrixException("Неизвестная ошибка", e);
            }
        }
        /// <summary>
        /// собственные числа для матрицы 3х3
        /// </summary>
        /// <returns>столбец собственных чисел</returns>
        public Matrix GetEigenValues()
        {
            if (this.Rows != 3 || this.Cols != 3) throw new MatrixException("Матрица должна быть 3х3");
            /*
             * посчитаем определитель
             * получим многочлен вида:
             * л^3 + a*л^2 + b*л + c
             */
            double a = -(this[0, 0] + this[1, 1] + this[2, 2]); //коэффициент при л^2;
            double b = this[0, 0] * this[1, 1] + this[0, 0] * this[2, 2] + this[1, 1] * this[2, 2] - this[2, 0] * this[0, 2] - this[1, 0] * this[0, 1] - this[2, 1] * this[1, 2]; //коэффициент при л;
            double c = this[0, 2] * this[2, 0] * this[1, 1] + this[1, 0] * this[0, 1] * this[2, 2] + this[2, 1] * this[1, 2] * this[0, 0] - this[0, 0] * this[1, 1] * this[2, 2] - this[1, 0] * this[2, 1] * this[0, 2] - this[2, 0] * this[0, 1] * this[1, 2]; //свободный член

            //решаем методом Виета
            double q = (Math.Pow(a, 2) - 3 * b) / 9;
            double r = (2 * Math.Pow(a, 3) - 9 * a * b + 27 * c) / 54;

            if (!(Math.Pow(r, 2) < Math.Pow(q, 3))) throw new MatrixException("У матрицы не 3 собственных числа");

            double t = Math.Acos(r / Math.Sqrt(q * q * q)) / 3; //через Pow(q, 3/2) не заработал
            Matrix l = new Matrix(3);//лямбда, л

            l[0] = -2 * Math.Sqrt(q) * Math.Cos(t) - a / 3;
            l[1] = -2 * Math.Sqrt(q) * Math.Cos(t + 2 * Math.PI / 3) - a / 3;
            l[2] = -2 * Math.Sqrt(q) * Math.Cos(t - 2 * Math.PI / 3) - a / 3;
            return l;
        }
        /// <summary>
        /// матрица из собственных векторов для текущей матрицы по собственым значениям
        /// </summary>
        /// <returns>матрицу из собственных векторов</returns>
        public Matrix GetMatrixFromEigenVector()
        {
            Matrix l = this.GetEigenValues();
            Matrix rez = this.GetEigenVector(l[0]);
            for (int j = 1; j < rez.Rows; j++)
                rez = Matrix.ExtendMatrix(rez, this.GetEigenVector(l[j]));
            return rez;
        }
        /// <summary>
        /// возвращает собственный вектор для собственного числа
        /// </summary>
        /// <param name="l">собственное число</param>
        public Matrix GetEigenVector(double l)
        {
            Matrix vector = new Matrix(this.Rows);
            Matrix y = new Matrix(this.Rows); 		// U*vector = y
            y[y.Rows - 1] = 1; 						//чтобы не вырождался

            Matrix matrix = this.Clone();
            for (int k = 0; k < this.Rows; k++)
                matrix[k, k] -= l; 					//отнимаем собственное значение

            Matrix L, U; 							//LU разложение
            matrix.GetLU(out L, out U);
            U[U.Rows - 1, U.Cols - 1] = 1; 			//чтобы не вырождался

            for (int i = U.Rows - 1; i >= 0; i--) 	//в обратном порядке
            {
                double summ = 0;
                for (int k = i + 1; k < U.Rows; k++)
                    summ += U[i, k] * vector[k];
                vector[i] = (y[i] - summ) / U[i, i];
            }

            //нормируем вектор
            double norma = Math.Sqrt((vector.Transpose() * vector)[0, 0]);
            vector /= norma;

            return vector;
        }
        /// <summary>
        /// Определитель матрицы
        /// </summary>
        public double GetDeterminant()
        {
            double rez = 1;
            Matrix L, U; 							//LU разложение
            this.GetLU(out L, out U);

            for (int i = 0; i < this.Rows; i++)
                rez *= U[i, i];
            return rez;
        }
        /// <summary>
        /// ортогональна ли матрица с ошибкой 1e-3
        /// </summary>
        public bool IsOrthogonal()
        {
            const double error = 1e-3F;
            Matrix matrix = this.Transpose() * this;
            for (int i = 0; i < matrix.Rows; i++)
                for (int j = 0; j < matrix.Cols; j++)
                {
                    if (i == j)
                    {
                        if (Math.Abs(matrix[i, j] - 1) > error) return false;
                    }
                    else
                    {
                        if (Math.Abs(matrix[i, j]) > error) return false;
                    }
                }
            return true;
        }
        /// <summary>
        /// сумма двух матриц
        /// </summary>
        /// <param name="mLeft">левая матрица</param>
        /// <param name="mRight">правая матрица</param>
        /// <returns>результат сложения</returns>
        private static Matrix Summ(Matrix mLeft, Matrix mRight)
        {
            if ((mLeft.Rows != mRight.Rows) ||
                 (mLeft.Cols != mRight.Cols))
                throw new MatrixException("У матриц дожна быть одинаковая размерность");
            Matrix mOut = new Matrix(mLeft.Rows, mLeft.Cols);
            for (int i = 0; i < mLeft.Rows; i++)
                for (int j = 0; j < mLeft.Cols; j++)
                    mOut[i, j] = mLeft[i, j] + mRight[i, j];
            return mOut;
        }
        private static double[] Summ(Matrix mLeft, double[] mRight)
        {
            double[] Result = new double[mRight.Length];
            for (int i = 0; i < mRight.Length; i++)
                Result[i] = mLeft[i] + mRight[i];
            return Result;
        }

        //
        //Создаем матрицу из диадного произведения векторов
        //
        public static Matrix MatrixFromDiadVector(double[] a, double[] b)
        {
            Matrix mOut = new Matrix(a.Length, a.Length);

            for (int i = 0; i < a.Length; i++)
            {
                for (int j = 0; j < a.Length; j++)
                {
                    mOut[i, j] = a[i] * b[j];
                }
            }

            return mOut;
        }
        //
        //Создаем матрицу из диадного произведения векторов
        //
        public static double[] VectorFromVectorOnMatrix(double[] a, Matrix b)
        {
            double[] mOut = new double[a.Length];

            for (int i = 0; i < a.Length; i++)
            {
                for (int j = 0; j < a.Length; j++)
                {
                    mOut[i] += a[j] * b[j, i];
                }
            }

            return mOut;
        }

        //
        //Конвертирует вектор в матрицу с 1 столбцом
        //
        public static Matrix ConvertToMatrix(double[] vectorIN)
        {
            Matrix mOut = new Matrix(vectorIN.Length);
            for (int i =0; i < vectorIN.Length; i++)
                mOut[i] = vectorIN[i];
            return mOut;
        }
        /// <summary>
        ///отрицательная матрица
        /// </summary>
        /// <returns>матрицу с элементами знаков, противоположных исходным</returns>
        private Matrix Negate()
        {
            Matrix mOut = new Matrix(this.Rows, this.Cols);
            for (int i = 0; i < this.Rows; i++)
                for (int j = 0; j < this.Cols; j++)
                    mOut[i, j] = -this[i, j];
            return mOut;
        }
        private static double[] Negate(Matrix mLeft, double[] mRight)
        {
            double[] Result = new double[mRight.Length];
            for (int i = 0; i < mRight.Length; i++)
                Result[i] = mLeft[i] - mRight[i];
            return Result;
        }
        private static double[] Negate(double[] mLeft, Matrix mRight)
        {
            double[] Result = new double[mLeft.Length];
            for (int i = 0; i < mLeft.Length; i++)
                Result[i] = mLeft[i] - mRight[i];
            return Result;
        }
        /// <summary>
        /// Рассчет обратной матрицы через LU разложение
        /// </summary>
        /// <returns>обратную матрицу</returns>
        public Matrix GetInvertible()
        {
            if (Rows != Rows)
                throw new MatrixException("Матрица должна быть квадратной");
            Matrix X = new Matrix(Rows, Cols);
            Matrix L, U;
            this.GetLU(out L, out U);

            for (int k = Rows - 1; k >= 0; k--)			//формулы из книги Вержбицкого "Основы численных методов"
            {
                {
                    double sum = 1;
                    for (int n = k + 1; n < Rows; n++)
                    {
                        sum -= U[k, n] * X[n, k];
                    }
                    X[k, k] = sum / U[k, k];
                }

                for (int i = k - 1; i >= 0; i--)
                {
                    double sum = 0;
                    for (int n = i + 1; n < Rows; n++)
                    {
                        sum += U[i, n] * X[n, k];
                    }
                    X[i, k] = -sum / U[i, i];
                }
                for (int j = k - 1; j >= 0; j--)
                {
                    double sum = 0;
                    for (int n = j + 1; n < Rows; n++)
                    {
                        sum += L[n, j] * X[k, n];
                    }
                    X[k, j] = -sum;
                }
            }
            return X;
        }
        /// <summary>
        /// умножение матриц
        /// </summary>
        /// <param name="mLeft">левая матрица</param>
        /// <param name="mRight">правая матрица</param>
        /// <returns>результат умножения</returns>
        public static Matrix Multiply(Matrix mLeft, Matrix mRight)
        {
            if (mLeft.Cols != mRight.Rows)
                throw new MatrixException("Умножение матриц невозможно");
            Matrix mOut = new Matrix(mLeft.Rows, mRight.Cols);
            for (int i = 0; i < mOut.Rows; i++)
                for (int j = 0; j < mOut.Cols; j++)
                    for (int k = 0; k < mLeft.Cols; k++)
                        mOut[i, j] += mLeft[i, k] * mRight[k, j];
            return mOut;
        }

        public static double[] Multiply(Matrix mLeft, double[] mRight)
        {
            if (mLeft.Cols != mRight.Length)
                throw new MatrixException("Умножение матриц невозможно");
            double[] mOut = new double[mLeft.Rows];
            for (int i = 0; i < mLeft.Rows; i++)
                for (int j = 0; j < mLeft.Cols; j++)
                    mOut[i] += mLeft[i, j] * mRight[j];
            return mOut;
        }

        /// <summary>
        /// Кососимметрическая матрица из вектора
        /// </summary>
        /// <param name="mLeft">левая матрица</param>
        /// <param name="mRight">правая матрица</param>
        /// <returns>результат умножения</returns>
        public static Matrix SkewSymmetricMatrix(double[] Vector)
        {
            Matrix mOut = new Matrix(3, 3);
            mOut[0, 1] = Vector[2];
            mOut[0, 2] = -Vector[1];
            mOut[1, 0] = -Vector[2];
            mOut[1, 2] = Vector[0];
            mOut[2, 0] = Vector[1];
            mOut[2, 1] = -Vector[0];

            return mOut;
        }

        /// <summary>
        /// Квадрат Кососимметрической матрица из вектора
        /// </summary>
        /// <param name="mLeft">левая матрица</param>
        /// <param name="mRight">правая матрица</param>
        /// <returns>результат умножения</returns>
        public static Matrix SkewSymmetricMatrixSquare(double[] Vector)
        {
            Matrix mOut = new Matrix(3, 3);
            mOut[0, 0] = -Vector[1] * Vector[1] - Vector[2] * Vector[2];
            mOut[1, 1] = -Vector[0] * Vector[0] - Vector[2] * Vector[2];
            mOut[2, 2] = -Vector[0] * Vector[0] - Vector[1] * Vector[1];
            mOut[0, 1] = Vector[0] * Vector[1];
            mOut[1, 0] = Vector[0] * Vector[1];
            mOut[0, 2] = Vector[0] * Vector[2];
            mOut[2, 0] = Vector[0] * Vector[2];
            mOut[1, 2] = Vector[1] * Vector[2];
            mOut[2, 1] = Vector[1] * Vector[2];

            return mOut;
        }

        public static Matrix DoA_eta_xi(double T)
        {
            Matrix mOut = new Matrix(3, 3);
            mOut[0, 0] = Math.Cos(SimpleData.U * T);
            mOut[1, 1] = Math.Cos(SimpleData.U * T);
            mOut[0, 1] = Math.Sin(SimpleData.U * T);
            mOut[1, 0] = -Math.Sin(SimpleData.U * T);
            mOut[2, 2] = 1.0;

            return mOut;
        }

        /// <summary>
        /// умножение каждого элемента на число(double)
        /// </summary>
        /// <param name="mIn">матрица</param>
        /// <param name="dK">число</param>
        /// <returns>результат умножения</returns>
        public static Matrix Multiply(Matrix mIn, double dK)
        {
            Matrix mOut = new Matrix(mIn.Rows, mIn.Cols);
            for (int i = 0; i < mOut.Rows; i++)
                for (int j = 0; j < mOut.Cols; j++)
                    mOut[i, j] = mIn[i, j] * dK;
            return mOut;
        }
        /// <summary>
        /// поиск решения неоднородной невырожденной СЛАУ: Ax = b
        /// </summary>
        /// <param name="A">матрица коэффициентов</param>
        /// <param name="B">вектор свободных членов</param>
        /// <returns>решение</returns>
        public static Matrix SolvingSystemsOfLinearEquations(Matrix A, Matrix B)
        {
            if ((A.Rows != A.Cols) && (B.Rows != A.Rows) && B.Cols != 1) throw new MatrixException("Некорректные данные для решения СЛАУ");
            if ((B.Transpose() * B)[0] == 0) return B;   //система однородна

            Matrix L, U;
            A.GetLU(out L, out U);

            double determ = 1;
            for (int k = 0; k < U.Rows; k++)
                determ *= U[k, k];
            if (Math.Abs(determ) < 1e-7) throw new MatrixException("Система вырождена"); //нулевой определитель
            /*
             * A = LU
             * LUx = b
             * 
             * Ly = b
             * Ux = y
             */
            Matrix X = new Matrix(B.Rows);
            Matrix Y = new Matrix(B.Rows);

            for (int i = 0; i < L.Rows; i++)		// Ly = b
            {
                Y[i] = B[i];
                for (int k = 0; k < i; k++)
                    Y[i] -= L[i, k] * Y[k];
            }
            for (int i = U.Rows - 1; i >= 0; i--)	// Ux = y
            {
                X[i] = Y[i];
                for (int k = i + 1; k < U.Rows; k++)
                    X[i] -= U[i, k] * X[k];
                X[i] /= U[i, i];
            }
            return X;
        }

        #region operators
        public static double[] operator +(Matrix mLeft, double[] mRight)
        {
            return Matrix.Summ(mLeft, mRight);
        }
        public static double[] operator +(double[] mLeft, Matrix mRight)
        {
            return Matrix.Summ(mRight, mLeft);
        }
        public static double[] operator -(Matrix mLeft, double[] mRight)
        {
            return Matrix.Negate(mLeft, mRight);
        }
        public static double[] operator -(double[] mLeft, Matrix mRight)
        {
            return Matrix.Negate(mRight, mLeft);
        }

        public static Matrix operator +(Matrix mLeft, Matrix mRight)
        {
            return Matrix.Summ(mLeft, mRight);
        }

        public static Matrix operator -(Matrix mIn)
        {
            return mIn.Negate();
        }

        public static Matrix operator -(Matrix mLeft, Matrix mRight)
        {
            return Matrix.Summ(mLeft, -mRight);
        }

        public static Matrix operator *(Matrix mLeft, Matrix mRight)
        {
            return Matrix.Multiply(mLeft, mRight);
        }

        public static double[] operator *(Matrix mLeft, double[] Vector)
        {
            return Matrix.Multiply(mLeft, Vector);
        }

        public static Matrix operator *(Matrix mIn, double dK)
        {
            return Matrix.Multiply(mIn, dK);
        }

        public static Matrix operator *(double dK, Matrix mIn)
        {
            return Matrix.Multiply(mIn, dK);
        }

        public static Matrix operator /(Matrix mIn, double dK)
        {
            return Matrix.Multiply(mIn, 1 / dK);
        }
        #endregion

        public IEnumerator GetEnumerator()
        {
            return m.GetEnumerator();
        }
    }

    class MatrixException : Exception
    {
        public MatrixException()
            : base()
        { }
        public MatrixException(string Message)
            : base(Message)
        { }
        public MatrixException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }
}
