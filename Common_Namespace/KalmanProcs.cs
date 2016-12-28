using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Common_Namespace
{
    public class KalmanProcs
    {
        public static void Make_F(double timeStep, Kalman_Vars KalmanVars, SINS_State SINSstate)
        {
            unsafe
            {
                fixed (double* _a = KalmanVars.Matrix_A, _f = KalmanVars.TransitionMatrixF)
                {
                    func(timeStep, _a, _f, SimpleData.iMx);
                }
            }


            // ----------------------------------------------------------//
            unsafe
            {
                fixed (double* _a = KalmanVars.Vertical_Matrix_A, _f = KalmanVars.Vertical_TransitionMatrixF)
                {
                    func(timeStep, _a, _f, SimpleData.iMx_Vertical);
                }
            }
        }



        public static void KalmanCorrection(Kalman_Vars KalmanVars, SINS_State SINSstate, SINS_State SINSstate_OdoMod)
        {
            SimpleOperations.NullingOfArray(KalmanVars.KalmanFactor);
            SimpleOperations.NullingOfArray(KalmanVars.StringOfMeasure);

            // --- Запускается коррекция по циклу по одной строке из матрицы H
            unsafe
            {
                fixed (double* _xm = KalmanVars.ErrorConditionVector_m, _xp = KalmanVars.ErrorConditionVector_p, _sm = KalmanVars.CovarianceMatrixS_m, _sp = KalmanVars.CovarianceMatrixS_p, _kf = KalmanVars.KalmanFactor)
                {
                    for (int t = 0; t < KalmanVars.cnt_measures; t++)
                    {
                        for (int i = 0; i < SimpleData.iMx; i++)
                            KalmanVars.StringOfMeasure[i] = KalmanVars.Matrix_H[t * SimpleData.iMx + i];

                        fixed (double* _h = KalmanVars.StringOfMeasure)
                        {
                            // --- Коррекция по измерениям
                            f0b(KalmanVars.Measure[t], _xm, _sm, _h, KalmanVars.Noize_Z[t] * KalmanVars.Noize_Z[t], _xp, _sp, _kf, SimpleData.iMx);

                            if (t < KalmanVars.cnt_measures - 1)
                            {
                                for (int i = 0; i < SimpleData.iMx; i++)
                                {
                                    _xm[i] = _xp[i];
                                    for (int j = 0; j < SimpleData.iMx; j++)
                                        _sm[i * SimpleData.iMx + j] = _sp[i * SimpleData.iMx + j];
                                }
                            }
                        }
                    }
                }
            }




            // ----------------------------------------------------------//
            // --------------Шаг коррекции для вертикального фильтра-----------------//
            // ----------------------------------------------------------//
            SimpleOperations.NullingOfArray(KalmanVars.KalmanFactor);
            SimpleOperations.NullingOfArray(KalmanVars.StringOfMeasure);

            unsafe
            {
                fixed (double* _xm = KalmanVars.Vertical_ErrorConditionVector_m, _xp = KalmanVars.Vertical_ErrorConditionVector_p, _sm = KalmanVars.Vertical_CovarianceMatrixS_m,
                    _sp = KalmanVars.Vertical_CovarianceMatrixS_p, _kf = KalmanVars.KalmanFactor)
                {
                    for (int t = 0; t < KalmanVars.Vertical_cnt_measures; t++)
                    {
                        for (int i = 0; i < SimpleData.iMx_Vertical; i++)
                            KalmanVars.StringOfMeasure[i] = KalmanVars.Vertical_Matrix_H[t * SimpleData.iMx_Vertical + i];

                        fixed (double* _h = KalmanVars.StringOfMeasure)
                        {
                            //Коррекция по измерениям
                            f0b(KalmanVars.Vertical_Measure[t], _xm, _sm, _h, KalmanVars.Vertical_Noize_Z[t] * KalmanVars.Vertical_Noize_Z[t], _xp, _sp, _kf, SimpleData.iMx_Vertical);

                            if (t < KalmanVars.Vertical_cnt_measures - 1)
                            {
                                for (int i = 0; i < SimpleData.iMx_Vertical; i++)
                                {
                                    _xm[i] = _xp[i];
                                    for (int j = 0; j < SimpleData.iMx_Vertical; j++)
                                        _sm[i * SimpleData.iMx_Vertical + j] = _sp[i * SimpleData.iMx_Vertical + j];
                                }
                            }
                        }
                    }
                }
            }
        }


        public static void KalmanForecast(Kalman_Vars KalmanVars, SINS_State SINSstate)
        {
            unsafe
            {
                fixed (double* _xm = KalmanVars.ErrorConditionVector_m, _xp = KalmanVars.ErrorConditionVector_p, _sm = KalmanVars.CovarianceMatrixS_m, _sp = KalmanVars.CovarianceMatrixS_p,
                    _f = KalmanVars.TransitionMatrixF, _sq = KalmanVars.CovarianceMatrixNoise)
                {
                    dgq0b(_xp, _sp, _f, _sq, _xm, _sm, SimpleData.iMx, SimpleData.iMq);
                }
            }
            SimpleOperations.CopyArray(KalmanVars.CovarianceMatrixS_p, KalmanVars.CovarianceMatrixS_m);
            SimpleOperations.CopyArray(KalmanVars.ErrorConditionVector_p, KalmanVars.ErrorConditionVector_m);



            // ----------------------------------------------------------//
            // -----------------Прогноз для вертикального фильтра--------------------//
            unsafe
            {
                fixed (double* _xm = KalmanVars.Vertical_ErrorConditionVector_m, _xp = KalmanVars.Vertical_ErrorConditionVector_p, _sm = KalmanVars.Vertical_CovarianceMatrixS_m,
                    _sp = KalmanVars.Vertical_CovarianceMatrixS_p, _f = KalmanVars.Vertical_TransitionMatrixF, _sq = KalmanVars.Vertical_CovarianceMatrixNoise)
                {
                    dgq0b(_xp, _sp, _f, _sq, _xm, _sm, SimpleData.iMx_Vertical, SimpleData.iMq_Vertical);
                }
            }
            SimpleOperations.CopyArray(KalmanVars.Vertical_CovarianceMatrixS_p, KalmanVars.Vertical_CovarianceMatrixS_m);
            SimpleOperations.CopyArray(KalmanVars.Vertical_ErrorConditionVector_p, KalmanVars.Vertical_ErrorConditionVector_m);
        }






        /*
*
*_________________________________________________________________
*
*                 *********
*                 *  f0b  *
*                 *********
*
*   f0b(z,xm,sm,h,r,xp,sp,kf,m) - подпрограмма         реализующа
*                                 алгоритм фильтра   Калмана   дл
*                                 этапа    коррекции    измерений.
*                                 (Оценка   плюс    ковариационные
*                                 соотношения).
*
*   Входные параметры:
*
*                m  - размерность системы (вектора состояния);
*                z  - измерение  z (скаляр);         _
*                xm - идентификатор априорной оценки Х(-) (m x 1);
*                sm - идентификатор      априорного     значени
*                     квадратного корня S(-) (m x m), хранящийся в форме вектора;
*                h  - идентификатор вектора измерения h (m x 1);
*                r  - идентификатор  дисперсии шума измерения  z
*                     (скаляр).
*
*   Выходные параметры:
*                                                             _
*                  xp - идентификатор апостериорной  оценки   Х(+)
*                       (m x 1);
*                  sp - идентификатор   апостериорного    значени
*                       квадратного корня S(+) (m x m), хранящийся в форме вектора;
*                  kf - идентификатор  калмановского  коэффициента
*                       усиления (m x 1).
*
*   ВНИМАНИЕ!!!
*   ВНИМАНИЕ!!!   Для  массивов   sm   и   sp   значимыми являютс
*   ВНИМАНИЕ!!!   только их верхнетреугольные области.
*   ВНИМАНИЕ!!!
*
*
*/

        unsafe static void f0b(double z, double* xm, double* sm, double* h, double r, double* xp, double* sp, double* kf, int m)
        {
            int i, j, ji, ii, m1;
            double y1, c, y, zm, al, al1, a;
            m1 = m + 1;
            y1 = *sm;
            c = *h;
            y = c * y1;
            zm = c * *xm;
            al = r + y * y;
            *kf = y1 * y;
            *sp = y1 * Math.Sqrt(r / al);
            for (i = 1; i < m; i++)
            {
                y = *(h + i);
                zm += y * *(xm + i);
                ii = m1 * i;
                y *= *(sm + ii);
                for (j = 0; j < i; j++) y += *(sm + j * m + i) * *(h + j);
                al1 = al + y * y;
                a = 1.0 / Math.Sqrt(al * al1);
                c = y * a;
                a *= al;
                al = al1;
                for (j = 0; j < i; j++)
                {
                    y1 = *(kf + j);
                    ji = j * m + i;
                    al1 = *(sm + ji);
                    *(sp + ji) = al1 * a - y1 * c;
                    *(kf + j) = y1 + al1 * y;
                }
                y1 = *(sm + ii);
                *(sp + ii) = y1 * a;
                *(kf + i) = y1 * y;
            }
            y = 1.0 / al;
            y1 = (z - zm) * y;
            for (i = 0; i < m; i++)
            {
                a = *(kf + i);
                *(xp + i) = *(xm + i) + a * y1;
                *(kf + i) = a * y;
            }
        }


        /*
*
*_________________________________________________________________
*
*                 ***********
*                 *  dgq0b  *
*                 ***********
*
*   dgq0b(xp,sp,f,sq,xm,sm,m,w) - подпрограмма         реализующа
*                                 алгоритм  фильтра   Калмана  дл
*                                 этапа прогноза.
*                                 (Оценка   плюс    ковариационные
*                                 соотношения).
*                                 Основу вычислительного алгоритма
*                                 составляет             процедура
*                                 ортогонализации  Грамма-Шмидта.
*
*
*   ВНИМАНИЕ!!!
*   ВНИМАНИЕ!!!   В модели задачи шум системы  q=B*u присутствует.
*   ВНИМАНИЕ!!!
*
*   Входные параметры:
*
*                m  - размерность системы (вектора состояния);
*                xp - идентификатор оценки вектора состояния  Х(+)
*                     (m x 1) до прогноза;
*                sp - идентификатор  значения  квадратного корн
*                     S(+) (m x m) до  прогноза, хранящийся в форме вектора;
*                f  - идентификатор переходной матрицы Ф(m x m), хранящийся в форме вектора.
*                sq - идентификатор    корня   sq(m x mq)     из
*                     ковариационной   матрицы  Q(m x m)    шума
*                     системы q(j)=B(j)*u(j), хранящийся в форме вектора:
*                                                       T
*                       sq(j) = B(j)*Sq(j) : sq(j)*sq(j) = Q(j).
*
*   Выходные параметры:
*
*                  xm - идентификатор  оценки  Х(-) (m x 1)  после
*                       прогноза;
*                  sm - идентификатор значения  квадратного  корн
*                       S(-)  (m x m) после  прогноза, хранящийся в форме вектора.
*
*
*   Служебные переменные:
*
*                  w  - одномерный (m x 1) массив.
*
*   ВНИМАНИЕ!!!
*   ВНИМАНИЕ!!!   Для  массивов   sm   и   sp   значимыми являютс
*   ВНИМАНИЕ!!!   только их верхнетреугольные области.
*   ВНИМАНИЕ!!!
*
*   ВНИМАНИЕ!!!
*   ВНИМАНИЕ!!!   В  процессе   работы    подпрограммы  массив  sq
*   ВНИМАНИЕ!!!   портится.
*   ВНИМАНИЕ!!!
*
*/
        unsafe static void dgq0b(double* xp, double* sp, double* f, double* sq, double* xm, double* sm, int m, int mq)
        {
            int i, j, k;
            double c, y, y1, dy;
            double[] w = new double[m];
            for (i = 0; i < m; i++)
                w[i] = 0.0;

            fixed (double* _w = w)
            {
                for (i = 0; i < m; i++)
                {
                    c = 0.0;
                    for (j = 0; j < m; j++)
                    {
                        y = 0.0;
                        for (k = 0; k <= i; k++)
                            y += *(sp + k * m + i) * *(f + j * m + k);

                        c += *(f + i * m + j) * *(xp + j);
                        *(sm + i * m + j) = y;
                    }
                    *(xm + i) = c;
                }
                for (k = m - 1; k > 0; k--)
                {
                    y = 0.0;
                    for (i = 0; i < m; i++)
                        _w[i] = 0.0;

                    for (i = 0; i < m; i++)
                    {
                        y1 = *(sm + i * m + k);
                        y += y1 * y1;
                    }
                    for (i = 0; i < mq; i++)
                    {
                        y1 = *(sq + k * mq + i);
                        y += y1 * y1;
                    }

                    y = Math.Sqrt(y);
                    dy = 1.0 / y;
                    for (j = 0; j < k; j++)
                    {
                        y1 = 0.0;
                        for (i = 0; i < m; i++)
                            y1 += *(sm + i * m + j) * *(sm + i * m + k);
                        for (i = 0; i < mq; i++)
                            y1 += *(sq + j * mq + i) * *(sq + k * mq + i);

                        y1 *= dy;
                        *(_w + j) = y1;

                        c = y1 * dy;
                        for (i = 0; i < m; i++)
                            *(sm + i * m + j) -= (*(sm + i * m + k)) * c;
                        for (i = 0; i < mq; i++)
                            *(sq + j * mq + i) -= (*(sq + k * mq + i)) * c;
                    }

                    for (i = 0; i < k; i++)
                        *(sm + i * m + k) = *(_w + i);

                    *(sm + m * k + k) = y;
                }
                c = 0.0;
                for (i = 0; i < m; i++)
                {
                    y = *(sm + i * m);
                    c += y * y;
                }
                for (i = 0; i < mq; i++)
                {
                    y = *(sq + i);
                    c += y * y;
                }
                *sm = Math.Sqrt(c);
            }
        }



        /*
        *
        *_________________________________________________________________
        *
        *                 **********
        *                 *  func  *
        *                 **********
        *
        *   func(dt,a,f,m) - подпрограмма вычисления переходной матрицы  f
        *                    (m x m)  линейной  динамической   системы  по
        *                    значению матрицы   a(m x m)   соответствующей
        *                    непрерывной системы.
        *                    (Используется   численный    метод    второго
        *                    порядка).
        *
        *
        *                    f = E + (dt * a) + 0.5 ( dt * dt * a * a)
        *
        *                    (E - единичная матрица)
        *
        *
        *   Входные параметры:
        *
        *                 m  - размерность матриц  a  и  f;
        *                 dt - квантование времени системы;
        *                 a  - двумерный   массив  (m x m)   -     матрица
        *                      непрерывной модели системы, хранящийся в форме вектора.
        *
        *   Выходные параметры:
        *
        *                 f  - двумерный   массив   (m x m)  -  переходна
        *                      матрица системы, хранящийся в форме вектора.
        *
        */

        unsafe static void func(double dt, double* a, double* f, int m)
        {
            int i, j, k, ij, im;
            double c, y;
            c = 0.5 * dt * dt;
            for (i = 0; i < m; i++)
            {
                im = i * m;
                for (j = 0; j < m; j++)
                {
                    y = 0.0;
                    for (k = 0; k < m; k++) y += *(a + im + k) * *(a + k * m + j);
                    ij = im + j;
                    y = dt * *(a + ij) + c * y;
                    if (i == j) y = 1.0 + y;
                    *(f + ij) = y;
                }
            }
        }
    }
}
