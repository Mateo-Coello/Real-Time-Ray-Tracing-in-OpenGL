bool hitBB(in Ray r, in Node node, in float nearestHit, in vec3 invDir)
{
    vec3 invDir = 1 / r.dir;
    vec3 tNear = (node.minB.xyz - ray.orig) * invDir;
    vec3 tFar  = (node.maxB.xyz - ray.orig) * invDir;
    vec3 tMin = min(tNear,tFar);
    vec3 tMax = max(tNear,tFar);
    float t0 = max(max(max(max(tMin,vec3(0.0)), vec3(tMin.x)), vec3(tMin.y)), vec3(tMin.z)).x;
    float t1 = min(min(min(min(tMax,vec3(0.0)), vec3(tMax.x)), vec3(tMax.y)), vec3(tMax.z)).x;
    return t0 <= t1;
}

HitRecord traceRay(in Ray r, in Interval rayT)
{
    HitRecord rec;
    rec.hit = false;
    float nearestHit = rayT.max;

    int nodesToVisit[64];
    int toVisitOffset = 0, currentNodeIndex = 0;

    vec3 invDir = 1/r.dir;
    vec3 dirIsNeg = lessThan(invDir, vec3(0));

    while(True) {
        Node node = objectsBVH[currentNodeIndex];
        if hitBB(node, r, nearestHit){
            if (node.count > 0){
                for(int i = 0; i < node.count; i++) {
                    PrimitiveInfo objInfo = objectIDs[i + content];
                    
                    if (objInfo.type == SPHERE && hitSphere(spheres[objInfo.idx], r, interval(0.001, nearestHit), rec)){
                        rec.hit = true;
                        nearestHit = rec.t;
                    }
                    else if (hitTriangle(triangles[objInfo.idx], r, interval(0.001, nearestHit), rec)){
                        rec.hit = true;
                        nearestHit = rec.t;
                    }
                }
                if(toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset]
            }
            else {
                if (dirIsNeg[node.splitAxis]){
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = content + 1;
                } 
                else {
                    nodesToVisit[toVisitOffset++] = content + 1;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        }
        else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesTovisit[--toVisitOffset]'
        }
    }
    return rec;
}