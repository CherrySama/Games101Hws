//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

// 函数用于对场景中的发光物体的随机采样，首先计算出所有发光物体的面积总和，然后在这个面积中随机选择一个位置作为采样点。
void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection shader_point = intersect(ray);
    if (!shader_point.happened) 
        return Vector3f();

    // 是否自发光
    if (shader_point.m->hasEmission()) 
        return shader_point.m->getEmission(); // 返回发光颜色

    Vector3f l_dir(0); // 直接光照
    Vector3f l_indir(0); // 间接光照

    switch(shader_point.m->getType())
    {
        case MIRROR:
        {
            if (get_random_float() > RussianRoulette) 
                return l_dir;
            Vector3f obj2nextobjdir = shader_point.m->sample(ray.direction, shader_point.normal).normalized();
            Vector3f p_deviation = (dotProduct(ray.direction, shader_point.normal) < 0) ? // 避免光线自相交
                    shader_point.coords + shader_point.normal * EPSILON :
                    shader_point.coords - shader_point.normal * EPSILON ;
            Ray obj2nextobjray(p_deviation, obj2nextobjdir);
            Intersection nextObjInter = intersect(obj2nextobjray);
            if (nextObjInter.happened)
            {
                float pdf = shader_point.m->pdf(ray.direction, obj2nextobjdir, shader_point.normal);
                if (pdf > EPSILON)
                {
                    l_indir = castRay(obj2nextobjray, depth + 1) 
                        * shader_point.m->eval(ray.direction, obj2nextobjdir, shader_point.normal) 
                        * dotProduct(obj2nextobjdir, shader_point.normal)
                        / (pdf * RussianRoulette);
                }
            }
            break;
        }
        default:
        {
            // 对光源积分
            Intersection lightInter;
            float lightPdf = 0.0f;
            sampleLight(lightInter, lightPdf);
            Vector3f obj2light = lightInter.coords - shader_point.coords; // obj -> light的向量
            Vector3f obj2lightDir = obj2light.normalized(); // obj -> light的向量
            float obj2lightPow = obj2light.x * obj2light.x + obj2light.y * obj2light.y + obj2light.z * obj2light.z; // 光线在传播中的衰减

            Vector3f p_deviation = (dotProduct(ray.direction, shader_point.normal) < 0) ? // 避免光线自相交
                    shader_point.coords + shader_point.normal * EPSILON :
                    shader_point.coords - shader_point.normal * EPSILON ;
            Ray obj2lightRay(p_deviation, obj2lightDir);
            Intersection t = intersect(obj2lightRay);
            if (t.distance - obj2light.norm() > -EPSILON)
            {
                l_dir = lightInter.emit * shader_point.m->eval(ray.direction, obj2lightDir, shader_point.normal) 
                    * dotProduct(obj2lightDir, shader_point.normal) 
                    * dotProduct(-obj2lightDir, lightInter.normal) 
                    / obj2lightPow / lightPdf;
            }

            if (get_random_float() > RussianRoulette) 
                return l_dir;
            // 间接光照
            Vector3f obj2nextobjdir = shader_point.m->sample(ray.direction, shader_point.normal).normalized();
            Ray obj2nextobjray(p_deviation, obj2nextobjdir);
            Intersection nextObjInter = intersect(obj2nextobjray);
            if (nextObjInter.happened && !nextObjInter.m->hasEmission())
            {
                float pdf = shader_point.m->pdf(ray.direction, obj2nextobjdir, shader_point.normal);
                if (pdf > EPSILON)
                {
                    l_indir = castRay(obj2nextobjray, depth + 1) // 递归调用，代表的是从当前交点发射新的光线，并获取该光线在所有物体上的反射后产生的光照贡献。
                        * shader_point.m->eval(ray.direction, obj2nextobjdir, shader_point.normal) 
                        * dotProduct(obj2nextobjdir, shader_point.normal)
                        / (pdf * RussianRoulette);
                }
            }           
            break;
        }
    }
    return l_dir + l_indir;
}
