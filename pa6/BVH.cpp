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

// 递归建造包围盒
// BVH 是直接取空间包围盒最长的轴，对物体按距离排序做二分，而 SAH 主要是基于面积公式和概率论的方式来计算包围盒
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    // 创建节点
    BVHBuildNode* node = new BVHBuildNode(); // root

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        // 遍历每一个object的AABB，然后再整合成一个大的AABB
        bounds = Union(bounds, objects[i]->getBounds());

    // 若只有一个object，然后以这个object创建叶节点
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }

    // 若有两个object，则把这两个object分别作为左右子节点（根节点不存放有关object的数据）
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    // 大于两个物体时
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            // 计算所有物体AABB中心点的并集，得到一个以所有物体中心点为基准的边界
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid()); // Centroid()得到box的中心
        // 得到覆盖范围内最大的那条轴
        int dim = centroidBounds.maxExtent();
        switch (SplitMethod::SAH) // choose a way to build AABB
        {
            case SplitMethod::NAIVE:
            {
                // 在该轴上进行升序排序
                switch (dim) {
                case 0: // x
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().x <
                            f2->getBounds().Centroid().x;
                    });
                    break;
                case 1: // y
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().y <
                            f2->getBounds().Centroid().y;
                    });
                    break;
                case 2: // z
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().z <
                            f2->getBounds().Centroid().z;
                    });
                    break;
                }
                // 对半分
                auto beginning = objects.begin();
                auto middling = objects.begin() + (objects.size() / 2);
                auto ending = objects.end();
                // 创建左右子树的向量
                auto leftshapes = std::vector<Object*>(beginning, middling);
                auto rightshapes = std::vector<Object*>(middling, ending);

                assert(objects.size() == (leftshapes.size() + rightshapes.size())); // 使用断言来确保分割后的对象总数与原来一样

                node->left = recursiveBuild(leftshapes);
                node->right = recursiveBuild(rightshapes);

                node->bounds = Union(node->left->bounds, node->right->bounds);
                break;
            } 
            case SplitMethod::SAH:
            {
                float totalSurfaceArea = centroidBounds.SurfaceArea();
                int bucketSize = 10;
                int optimalSplitIndex = 0; // 最佳分割点的索引
                float minCost = std::numeric_limits<float>::infinity();
                // 在该轴上进行升序排序
                switch (dim) {
                case 0: // x
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().x <
                            f2->getBounds().Centroid().x;
                    });
                    break;
                case 1: // y
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().y <
                            f2->getBounds().Centroid().y;
                    });
                    break;
                case 2: // z
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().z <
                            f2->getBounds().Centroid().z;
                    });
                    break;
                }
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
                // when you find the split index, next step is to do what BVH do
                // 对半分
                auto beginning = objects.begin();
                auto middling = objects.begin() + (objects.size() * optimalSplitIndex / bucketSize); //划分点选为当前最优桶的位置
                auto ending = objects.end();
                // 创建左右子树的向量
                auto leftshapes = std::vector<Object*>(beginning, middling);
                auto rightshapes = std::vector<Object*>(middling, ending);

                assert(objects.size() == (leftshapes.size() + rightshapes.size())); // 使用断言来确保分割后的对象总数与原来一样

                node->left = recursiveBuild(leftshapes);
                node->right = recursiveBuild(rightshapes);

                node->bounds = Union(node->left->bounds, node->right->bounds);
                break;
            }
        }
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