#ifndef GO2GOAL_MATRIX_INCLUDE_HEADER
#define GO2GOAL_MATRIX_INCLUDE_HEADER

#include <initializer_list>
#include <vector>

#define ROS_INTEGRATED

#ifdef ROS_INTEGRATED
#include "ros/ros.h"
#endif

template<typename T>
class MatrixColumn
{
    private:
        T* data;
        int sz;
    public:
        
        template<typename TM>
        friend class Matrix;
        MatrixColumn():data(nullptr){};
        MatrixColumn(int s){Init(s);};
        void Init(int s)
        {
            sz = s;
            if(s > 0)
            {
                data = new T[s];
                memset(data,0,sizeof(T)*s);

            }
            else
                data = nullptr;
        }
        MatrixColumn(std::initializer_list<T> m):MatrixColumn(m.size()) //Single Array
        {
            sz = m.size();
            //this->MatrixColumn<T>(sz);
            T* p = data;
            for(const T& i : m)
            {
                *p = i;
                p++;
            }
        }
        MatrixColumn(const MatrixColumn& cpy)
        {
            // if(cpy.sz != this->sz)
            // {
            //     #ifdef ROS_INTEGRATED
            //     ROS_INFO("(Copy MatrixCol) Matrix differenet size ERROR");
            //     ROS_INFO("CpySize : %d , ThisSize : %d",cpy.sz,this->sz);
            //     #endif
            //     throw "Matrix differenet size ERROR";
            // }
            sz = cpy.sz;
            data = new T[sz];
            memcpy(data,cpy.data,sizeof(T)*sz);
            #ifdef ROS_INTEGRATED
            ROS_INFO("Success Copy MatrixCol");
            #endif
        }
        ~MatrixColumn()
        {
            delete []data;
        }
        T& operator[](int idx)
        {
            if(idx < sz && idx >= 0)
                return data[idx];
            #ifdef ROS_INTEGRATED
            ROS_INFO("([]MatrixCol) Out of Range ERROR");
            #endif
            throw "Out of Range ERROR";
            return data[0];
        }
        MatrixColumn& operator=(const MatrixColumn& m)
        {
            if(this != &m)
            {
                if(m.sz != this->sz)
                {
                    #ifdef ROS_INTEGRATED
                    ROS_INFO("(=MatrixCol&) Matrix differenet size ERROR");
                    #endif
                    throw "Matrix differenet size ERROR";
                }
                    
                memcpy(this->data,m.data,sizeof(T)*this->sz);
            }
            return *this;
        }
        MatrixColumn& operator=(MatrixColumn&& m)
        {
            if(this != &m)
            {
                if(m.sz != this->sz)
                {
                    #ifdef ROS_INTEGRATED
                    ROS_INFO("(=MatrixCol&&) Matrix differenet size ERROR");
                    #endif
                    throw "Matrix differenet size ERROR";
                }
                memcpy(this->data,m.data,sizeof(T)*this->sz);
            }
            return *this;
        }
};

template<typename T>
class Matrix
{
    private:
        std::vector< MatrixColumn<T> > data;
        int cols;
        int rows;
        /* data */
    public:
        Matrix(int ,int );
        Matrix(std::initializer_list<T> m);//Single Array
        Matrix(std::initializer_list< std::initializer_list<T> > m);//2D Array
        ~Matrix();
        Matrix(Matrix<T> &);
        MatrixColumn<T>& operator[](const int m);
        Matrix<T> Transpose();
        Matrix<T> operator+(const T m);
        Matrix<T> operator+(Matrix<T>& m);
        Matrix<T> operator-(const T m);
        Matrix<T> operator-(Matrix<T>& m);
        Matrix<T>& operator=(Matrix<T>& m);
        Matrix<T>& operator=(Matrix<T>&& m);
        Matrix<T> operator*(const T m);
        Matrix<T> operator*(Matrix<T>& m);
        Matrix<T> dot(Matrix<T>& m);
        void PrintMatrix();
        T normf();
};

template<typename T>
Matrix<T>::Matrix(int rows,int cols)
{
    this->cols = cols;
    this->rows = rows;
    if(cols*rows == 0)
        return;
    else
    {
        data.reserve(this->rows);
        for(int i = 0;i < this->rows;i++)
            data[i].Init(this->cols);
    }
    #ifdef ROS_INTEGRATED
    ROS_INFO("New Matrix with Size ( %d , %d )",this->rows,this->cols);
    #endif
}

template<typename T>
Matrix<T>::Matrix(std::initializer_list<T> m)//Single Array
{
    this->rows = 1;
    this->cols = m.size();
    data.push_back( MatrixColumn<T>(m) );
    #ifdef ROS_INTEGRATED
    ROS_INFO("New Matrix with Size ( %d , %d )",this->rows,this->cols);
    #endif
}

template<typename T>
Matrix<T>::Matrix(std::initializer_list< std::initializer_list<T> > m)//2D Array
{
    this->rows = m.size();
    std::initializer_list<int>::iterator it;

    for(auto i : m)
    {
        data.push_back( MatrixColumn<T>(i) );
    }
    this->cols = data[0].sz;
    #ifdef ROS_INTEGRATED
    ROS_INFO("New Matrix with Size ( %d , %d )",this->rows,this->cols);
    #endif
}

template<typename T>
Matrix<T>::~Matrix(){}

template<typename T>
Matrix<T>::Matrix(Matrix<T> &m)
{
    // if(this->cols != m.cols || this->rows != m.rows)
    // {
    //     #ifdef ROS_INTEGRATED
    //     ROS_INFO("(Copy Matrix) Matrix differenet size ERROR");
    //     #endif
    //     throw "Matrix differenet size ERROR";
    // }
    this->rows = m.rows;
    data.reserve(this->rows);
    for(int i = 0;i < this->rows;i++)
        data[i] = m[i];
    this->cols = data[0].sz;
    #ifdef ROS_INTEGRATED
    ROS_INFO("Success Copy Matrix");
    #endif
    #ifdef ROS_INTEGRATED
    ROS_INFO("New Matrix with Size ( %d , %d )",this->rows,this->cols);
    #endif
}

template<typename T>    
MatrixColumn<T>& Matrix<T>::operator[](const int m)
{
    if(m < this->rows && m >= 0)
        return data[m];
    
    #ifdef ROS_INTEGRATED
    ROS_INFO("([]Matrix) Matrix Out of range ERROR");
    #endif
    throw "Matrix Out of range ERROR";

    return data[0];
}

template<typename T>
Matrix<T> Matrix<T>::Transpose()
{
    Matrix<T> rtv(this->cols,this->rows);
    for(int i = 0;i < this->rows;i++)
        for(int j = 0;j < this->cols;j++)
            rtv[j][i] = data[i][j];
    return rtv;
}

template<typename T>
Matrix<T> Matrix<T>::operator+(const T m)
{
    Matrix rtv(this->rows,this->cols);

    for(int i = 0;i < this->rows;i++)
        for(int j = 0;j < this->cols;j++)
            rtv[i][j] = data[i][j] + m;
    return rtv;
}
template<typename T>
Matrix<T> Matrix<T>::operator+(Matrix<T>& m)
{
    if(this->cols != m.cols || this->rows != m.rows)
    {
        #ifdef ROS_INTEGRATED
        ROS_INFO("(+mATRIX) Matrix differenet size ERROR");
        #endif
        throw "Matrix differenet size ERROR";
    }

    Matrix rtv(this->rows,this->cols);
    
    for(int i = 0;i < this->rows;i++)
        for(int j = 0;j < this->cols;j++)
            rtv =  data[i][j] + m[i][j];
    return rtv;
}

template<typename T>
Matrix<T> Matrix<T>::operator-(const T m)
{
    Matrix rtv(this->rows,this->cols);

    for(int i = 0;i < this->rows;i++)
        for(int j = 0;j < this->cols;j++)
            rtv[i][j] = data[i][j] - m;
    return rtv;
}
template<typename T>
Matrix<T> Matrix<T>::operator-(Matrix<T>& m)
{
    if(this->cols != m.cols || this->rows != m.rows)
    {
        #ifdef ROS_INTEGRATED
        ROS_INFO("(-Matrix) Matrix differenet size ERROR");
        #endif
        throw "Matrix differenet size ERROR";
    }

    Matrix rtv(this->rows,this->cols);
    
    for(int i = 0;i < this->rows;i++)
    {
        // const MatrixColumn<T> mc(m[i]);
        // mc = m[i];
        for(int j = 0;j < this->cols;j++)
            rtv[i][j] =  data[i][j] - m[i][j];
    }
    
    #ifdef ROS_INTEGRATED
    ROS_INFO("(-Matrix) Success");
    #endif

    return rtv;
}

template<typename T>
Matrix<T>& Matrix<T>::operator=(Matrix<T>& m)
{
    if(this != &m)
    {
        if(this->cols != m.cols || this->rows != m.rows)
        {
            #ifdef ROS_INTEGRATED
            ROS_INFO("(=Matrix&) Matrix differenet size ERROR");
            #endif
            throw "Matrix differenet size ERROR";
        }

        #ifdef ROS_INTEGRATED
        ROS_INFO("(=Matrix) Test&");
        #endif
        for(int i = 0;i < this->rows;i++)
            for(int j = 0;j < this->cols;j++)
                data[i][j] = m[i][j];
    }
    #ifdef ROS_INTEGRATED
    ROS_INFO("(=Matrix) Success");
    #endif
    return (*this);
}

template<typename T>
Matrix<T>& Matrix<T>::operator=(Matrix<T>&& m)
{
    if(this != &m)
    {
        if(this->cols != m.cols || this->rows != m.rows)
        {
            #ifdef ROS_INTEGRATED
            ROS_INFO("(=Matrix&&) Matrix differenet size ERROR");
            ROS_INFO("ThisSize : %d,%d , mSize : %d,%d",this->rows,this->cols,m.rows,m.cols);
            #endif
            throw "Matrix differenet size ERROR";
        }

        #ifdef ROS_INTEGRATED
        ROS_INFO("(=Matrix) Test&&");
        #endif

        for(int i = 0;i < this->rows;i++)
            for(int j = 0;j < this->cols;j++)
                data[i][j] = std::move(m[i][j]);
    }
    
    return (*this);
}

template<typename T>
Matrix<T> Matrix<T>::operator*(const T m)
{
    Matrix rtv(this->rows,this->cols);
    
    for(int i = 0;i < this->rows;i++)
        for(int j = 0;j < this->cols;j++)
            rtv[i][j] = data[i][j] * m;
    return rtv;

}

template<typename T>
Matrix<T> Matrix<T>::operator*(Matrix<T>& m)
{
    if(this->cols != m.cols || this->rows != m.rows)
    {
        #ifdef ROS_INTEGRATED
        ROS_INFO("(*Matrix) Matrix differenet size ERROR");
        #endif
        throw "Matrix differenet size ERROR";
    }

    Matrix rtv(this->rows,this->cols);
    
    for(int i = 0;i < this->rows;i++)
        for(int j = 0;j < this->cols;j++)
            rtv[i][j] = data[i][j] * m[i][j];
    return rtv;

}

template<typename T>
Matrix<T> Matrix<T>::dot(Matrix<T>& m)
{

    if(this->cols != m.rows)
    {
        #ifdef ROS_INTEGRATED
        ROS_INFO("Matrix size cant be dotted ERROR");
        ROS_INFO("ThisSize : %d,%d , mSize : %d,%d",this->rows,this->cols,m.rows,m.cols);
        #endif
        throw "Matrix size cant be dotted ERROR";
    }
    Matrix rtv(this->rows,m.cols);

    for(int i = 0;i < this->rows;i++)
        for(int j = 0;j < m.cols;j++)
            for(int k = 0;k < this->cols;k++)
            {
                T dataIK    = data[i][k];
                T mKJ       = m[k][j];
                rtv[i][j] += dataIK * mKJ;
            }

    return rtv;
}

template<typename T>
T Matrix<T>::normf()
{
    T rtv = 0;
    for(int i = 0;i < this->rows;i++)
        for(int j = 0;j < this->cols;j++)
            rtv += this->data[i][j] * this->data[i][j];
    return sqrt(rtv);
}

template<typename T>
void Matrix<T>::PrintMatrix()
{
    for(int i = 0;i < this->rows;i++)
    {
        for(int j = 0;j < this->cols;j++)
            printf("%lf ",data[i][j]);
        printf("\n");
    }
        
}


#endif