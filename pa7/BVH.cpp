#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
        // 使用SAH加速
        float totalSurfaceArea = centroidBounds.SurfaceArea();
        int bucketSize = 10;
        int optimalSplitIndex = 0; // 最佳分割点的索引
        float minCost = std::numeric_limits<float>::infinity();
        // find the split index that minimizes the SAH cost
        for (int i = 1; i < bucketSize; i++)
        {
            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() * i / bucketSize);
            auto ending = objects.end();
            auto leftshapes = std::vector<Object *>(beginning, middling);
            auto rightshapes = std::vector<Object *>(middling, ending);
            //求左右包围盒:
            Bounds3 leftBounds, rightBounds;
            for (int j = 0; j < leftshapes.size(); ++j)
                leftBounds = Union(leftBounds, leftshapes[j]->getBounds().Centroid());
            for (int j = 0; j < rightshapes.size(); ++j)
                rightBounds = Union(rightBounds, rightshapes[j]->getBounds().Centroid());
            float SA = leftBounds.SurfaceArea(); //SA
            float SB = rightBounds.SurfaceArea(); //SB
            float cost = 0.125 + (leftshapes.size() * SA + rightshapes.size() * SB) / totalSurfaceArea; //计算花费
            if (cost < minCost)
            {
                minCost = cost;
                optimalSplitIndex = i;
            }
        }
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() * optimalSplitIndex / bucketSize); //划分点选为当前最优桶的位置
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

// 使用BVH结构求ray打到的离光线原点最近的交点
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection isect;

    std::array<int, 3> dirIsNeg;
    dirIsNeg[0] = int(ray.direction.x >= 0);
    dirIsNeg[1] = int(ray.direction.y >= 0);
    dirIsNeg[2] = int(ray.direction.z >= 0);

    // 如果没有交点就没必要进行进一步的细分，直接返回当前的intersection
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
    {
        return isect;
    }

    //如果是叶子节点，直接返回。
    if (node->left == nullptr && node->right == nullptr)
    {
        isect = node->object->getIntersection(ray);
        return isect;
    }

    // 深度优先遍历
    auto hit1 = getIntersection(node->left, ray);
    auto hit2 = getIntersection(node->right, ray);

    /*
    取光线打到最近的物体，可能出现的情况：
    1. 左右两个子节点里，一个打到了，一个没达到
    2. 都打到了，取一个更近的物体
    */
    return hit1.distance < hit2.distance ? hit1 : hit2;
}


void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf){
    if(node->left == nullptr || node->right == nullptr){
        node->object->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }
    if(p < node->left->area) getSample(node->left, p, pos, pdf);
    else getSample(node->right, p - node->left->area, pos, pdf);
}

void BVHAccel::Sample(Intersection &pos, float &pdf){
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);
    pdf /= root->area;
}