//
// Created by LEI XU on 5/13/19.
//
#pragma once
#ifndef RAYTRACING_OBJECT_H
#define RAYTRACING_OBJECT_H

#include "Vector.hpp"
#include "global.hpp"
#include "Bounds3.hpp"
#include "Ray.hpp"
#include "Intersection.hpp"

class Object
{
public:
    Object() {}
    virtual ~Object() {}
    virtual bool intersect(const Ray& ray) = 0; // 用于判断一条射线是否与该物体相交
    virtual bool intersect(const Ray& ray, float &, uint32_t &) const = 0; // 也是用于检测射线与物体是否相交，但此函数还会返回交点的参数化表示和相交点索引
    virtual Intersection getIntersection(Ray _ray) = 0; // 返回射线与该物体的交点信息
    // 该函数用于获取物体表面的属性，如表面的法线、纹理坐标等
    virtual void getSurfaceProperties(const Vector3f &, const Vector3f &, const uint32_t &, const Vector2f &, Vector3f &, Vector2f &) const = 0;
    virtual Vector3f evalDiffuseColor(const Vector2f &) const =0; // 评估物体在特定纹理坐标下的漫反射颜色
    virtual Bounds3 getBounds()=0; // 返回物体的边界框
    virtual float getArea()=0; // 返回物体的表面积，每一个形状的计算方法都可以不一样
    virtual void Sample(Intersection &pos, float &pdf)=0; // 从物体表面采样一个点，用于光源采样。`pos` 参数是采样点的信息，`pdf` 是该点的概率密度函数值。
    virtual bool hasEmit()=0; // 判断该物体是否发光，也就是是否为光源
};



#endif //RAYTRACING_OBJECT_H
