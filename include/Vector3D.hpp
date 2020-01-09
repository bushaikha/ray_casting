#pragma once 

#include <cstdint>

template <class T>  

class Vector3D
{
        public:
        
                T x = 0;
                T y = 0;
                T z = 0;

                Vector3D<T> operator + (Vector3D<T> obj)
                {
                        Vector3D<T> res;
                        res.x = x + obj.x;
                        res.y = y + obj.y;
                        res.z = z + obj.z;
                        return res;
                }
                Vector3D<T> operator - (Vector3D<T> obj)
                {
                        Vector3D<T> res;
                        res.x = x - obj.x;
                        res.y = y - obj.y;
                        res.z = z - obj.z;
                        return res;
                }
                Vector3D<T> operator * (Vector3D<T> obj)
                {
                        Vector3D<T> res;
                        res.x = x * obj.x;
                        res.y = y * obj.y;
                        res.z = z * obj.z;
                        return res;
                }
                Vector3D<T> operator * (T tmp)
                {
                        Vector3D<T> res;
                        res.x = this->x * tmp;
                        res.y = this->y * tmp;
                        res.z = this->z * tmp;
                        return res;
                }
                Vector3D<T> operator / (T tmp)
                {
                        Vector3D<T> res;
                        res.x = this->x / tmp;
                        res.y = this->y / tmp;
                        res.z = this->z / tmp;
                        return res;
                }
                void operator = (T tmp)
                {
                        this->x = tmp;
                        this->y = tmp;
                        this->z = tmp;
                }

                template <typename M>
                operator Vector3D<M>()
                {
                        Vector3D<M> tmp;
                        tmp.x = M(this->x);
                        tmp.y = M(this->y);
                        tmp.z = M(this->z);

                        return tmp;
                }

};