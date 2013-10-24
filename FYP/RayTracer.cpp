#include "RayTracer.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#define EPSILON 0.00001 
#define SPECULAR_N 64

RayTracer::RayTracer(RenderManager* m, int tid) : manager(m), tracerId(tid) {
}

void RayTracer::run() {
    while (manager->isRendering() && (task = manager->getTask()) != NULL) {
        Vec3 color = traceRay(task);
        manager->setPixelData(task->x, task->y, color * task->p);
    }
    emit completed();
}

Vec3 RayTracer::traceRay(RenderNode* n)
{
	// max depth
	if (n->depth >= manager->maxDepth || n->p < manager->threshold)
	{
		return Vec3(0.f);
	}
    Scene* scene = manager->getScene();

	Intersection* intc = scene->intersect(n->ray);
	// no hit
	if (intc == NULL) {
		return Vec3(0.25f);
    }

	Vec3 point = n->ray->at(intc->t);
	
	Material* mat = intc->mat;
	Vec3 I = Vec3(0.f);
	Vec3 AE = Vec3(mat->ke);

	//ambient
	if (mat->isTransmissive)
	{
        AE += scene->ambient * mat->ka * (1.0f - mat->ior);
    } 
	else 
	{
        AE += scene->ambient * mat->ka;
    }

	I += AE;
    
	for (Light* l : scene->lights)
	{
		Vec3 atten = l->getColor(point) * l->shadowAttenuation(point) * l->distanceAttenuation(point);
		Vec3 L = l->getDirection(point);
		
        Vec3 normal;
        
        if (mat->displacementMap) {
            if (intc->texCoord.x >= 1 || intc->texCoord.x <= 0
                || intc->texCoord.y >= 1 || intc->texCoord.y <= 0) {
                normal = intc->normal;
            } else {
            int x = mat->displacementMap->width() * intc->texCoord.x;
            int y = mat->displacementMap->height() * intc->texCoord.y;
            
            QColor normalColor = mat->displacementMap->pixel(x, y);
            Vec3 perturb = Vec3(normalColor.red() / 255.0 - 0.5, normalColor.green() / 255.0 - 0.5,
                normalColor.blue() / 255.0 - 0.5);

            // printf("%.3f\t\t%.3f\t\t%.3f\n%.3f\t\t%.3f\t\t%.3f\n%.3f\t\t%.3f\t\t%.3f\t\t\n\n", intc->normal.x, intc->normal.y, intc->normal.z, intc->tangent.x, intc->tangent.y, intc->tangent.z, intc->bitangent.x, intc->bitangent.y, intc->bitangent.z);
            // printf("%.3f\t\t%.3f\t\t%.3f\n", perturb.x, perturb.y, perturb.z);
            
            normal = intc->normal * perturb.z + intc->tangent * perturb.x + intc->tangent * perturb.y;
            normal.normalize();
            }
        } else {
            normal = intc->normal;
        }
        
        float NL = dot(normal, L);

        Vec3 diffuse;
        if (mat->diffuseMap != NULL) // has diffuse map
		{
			Vec3 at = atten + AE * Vec3(1 / scene->lights.size());
            if (intc->texCoord.x >= 1 || intc->texCoord.x <= 0
                || intc->texCoord.y >= 1 || intc->texCoord.y <= 0) {
                diffuse = (at * mat->kd * NL);
            } else {
                int x = mat->diffuseMap->width() * intc->texCoord.x;
                int y = mat->diffuseMap->height() * intc->texCoord.y;
                
                QColor diffuseColor = mat->diffuseMap->pixel(x, y);
                diffuse = l->energy * at * Vec3(diffuseColor.red() / 255.0, diffuseColor.green() / 255.0, diffuseColor.blue() / 255.0) * NL;
            }
        }
		else
		{
            diffuse = (atten * mat->kd * NL);
        }
        
        diffuse.clamp();
        I += diffuse;
        
		//specular
		Vec3 R = intc->normal * (2 * NL) - L;
		double RV = -dot(R, n->ray->dir);
        
        Vec3 ks;
        
        if (mat->specularMap != NULL) { // has specular map
            if (intc->texCoord.x >= 1 || intc->texCoord.x <= 0
                || intc->texCoord.y >= 1 || intc->texCoord.y <= 0) {
                ks = mat->ks;
            } else {
            int x = mat->specularMap->width() * intc->texCoord.x;
            int y = mat->specularMap->height() * intc->texCoord.y;
            
            QColor specularity = mat->specularMap->pixel(x, y);
            ks = Vec3(specularity.red() / 255.0);
            }
        } else {
            ks = mat->ks;
        }
        
        I += (atten * pow(RV, SPECULAR_N)) * ks;
	}

    I *= 1 - (mat->reflectFactor + (1 - mat->alpha)) * n->p;
     
    // reflection
    const float NL = -dot(intc->normal, n->ray->dir);
    float reflGloss = 1 - mat->reflectGloss;
	
    if (abs(mat->reflectFactor > EPSILON)) { // reflective
        Vec3 refl = intc->normal * (2 * NL) + n->ray->dir;
        
        if (abs(reflGloss) < EPSILON) { // no glossy reflection
            Ray* R = new Ray(point, refl);
            RenderNode* r = new RenderNode(R, n->x, n->y, n->depth + 1, mat->reflectFactor * n->p);
            manager->addTask(r);
        } else { // glossy
            for (int i = 0; i < 10; i++) {
                Ray* R = new Ray(point, refl.randomize(reflGloss));
                RenderNode* r = new RenderNode(R, n->x, n->y, n->depth + 1, mat->reflectFactor * n->p * 0.1f);
                manager->addTask(r);
            }
        }
    }

    //refraction
    float refrGloss = 1 - mat->refractGloss;
	if (abs(mat->alpha - 1) > EPSILON) // alpha is not 1, refractive
	{
        // Vec3 refraction;
        float pn;
        
        if (NL > 0)
        {
            pn = mat->ior_inverse;
            float LONG_TERM = pn * NL - sqrt(1 - pn * pn * (1 - NL * NL));
            Vec3 refr = intc->normal * LONG_TERM + n->ray->dir * pn;
            if (abs(refrGloss) < EPSILON) { // no glossy refraction
                Ray* T = new Ray(point, refr);
                RenderNode* t = new RenderNode(T, n->x, n->y, n->depth + 1, (1 - mat->alpha) * n->p);
                manager->addTask(t);
            } else { // glossy
                for (int i = 0; i < 10; i++) {
                    Ray* T = new Ray(point, refr.randomize(refrGloss));
                    RenderNode* t = new RenderNode(T, n->x, n->y, n->depth + 1, (1 - mat->alpha) * n->p * 0.1f);
                    manager->addTask(t);
                }
            }
        }
        else
        {
            pn = mat->ior;
            if (1 - pn * pn * (1 - NL * NL) < EPSILON)
            {
                return I;
            }
            
            float LONG_TERM = -(pn * (-NL) - sqrt(1 - pn * pn * (1 - NL * NL)));
            Vec3 refr = intc->normal * LONG_TERM + n->ray->dir * pn;
            if (abs(refrGloss) < EPSILON) { // no glossy refraction
                Ray* T = new Ray(point, refr);
                RenderNode* t = new RenderNode(T, n->x, n->y, n->depth + 1, (1 - mat->alpha) * n->p);
                manager->addTask(t);
            } else { // glossy
                for (int i = 0; i < 10; i++) {
                    Ray* T = new Ray(point, refr.randomize(refrGloss));
                    RenderNode* t = new RenderNode(T, n->x, n->y, n->depth + 1, (1 - mat->alpha) * n->p * 0.1f);
                    manager->addTask(t);
                }
            }
        }
    }

    return I;
}
