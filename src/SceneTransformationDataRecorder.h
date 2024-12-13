#ifndef AFF_SCENETRANSFORMATIONDATARECORDER_H
#define AFF_SCENETRANSFORMATIONDATARECORDER_H

#include "ComponentBase.h"
#include <json.hpp>
#include <string>
#include <deque>
#include <Rcs_graph.h>
#include "ActionScene.h"
#include <set>

namespace aff
{

/*! \brief Structure to represent a parent-child transformation.
*
* This structure stores the relative transformation between a parent and child body.
* - parent:  The name of the parent body.
* - child: The name of the child body.
* - relativeTransformation: The relative transformation between the parent and child bodies.
*/

struct BodyTransformation
{
    std::string parent; // Name of the parent body
    std::string child; // Name of the child body
    HTr relativeTransformation; // Relative transformation (rotation+translation)

    /*! \brief Default constructor.
    */
    BodyTransformation() = default;

    /*! \brief Constructor to initialize a parent-child transformation.
    * 
    * \param[in] parent_ Name of the parent body.
    * \param[in] child_ Name of the child body.
    * \param[in] relativeTrf Relative transformation between the parent and child bodies.
    */
    BodyTransformation(const std::string& parent_, const std::string& child_, const HTr& relativeTrf)
        : parent(parent_), child(child_), relativeTransformation(relativeTrf)
    {
        // Initialize transformation values
        HTr_setIdentity(&relativeTransformation);
        HTr_copy(&relativeTransformation, &relativeTrf);
    }
};

/*! \brief Structure to represent all transformations at a specific time
*
* This structure stores the time and the selected parent-child transformations at that time.
*
*/
struct TransformationRecord
{
    double time; // Time of the recorded transformations
    std::vector<BodyTransformation> transformations; // List of all parent-child transformations at that time.

    /*! \brief Default constructor.
    */
    TransformationRecord() = default;

    /*! \brief Constructor to initialize a transformation record at a specific time.
    *
    * \param[in] t Time of the recorded transformations.
    */
    TransformationRecord(double t)
        : time(t)
    {
    }

    /*! \brief Add a parent-child transformation to the record.
    *
    * \param[in] parent Name of the parent body.
    * \param[in] child Name of the child body.
    * \param[in] relativeTrf Relative transformation between the parent and child bodies.
    * 
    */
    void addTransformation(const std::string& parent, const std::string& child, const HTr& relativeTrf)
    {
        transformations.emplace_back(parent, child, relativeTrf);
    }
};


class SceneTransformationDataRecorder : public ComponentBase
{
public:

    /*! \brief Constructs SceneTransformationDataRecorder
    *  The `SceneTransformationDataRecorder` tracks transformations of bodies in a scene over time.
    *  It records the relative transformations between bodies.
    * 
    * \param[in] parent Entity managing the event subscriptions.
    * \param[in] maxNumRecords_ Maximum number of transformation records to store.
    * 
    */
    SceneTransformationDataRecorder(EntityBase* parent, size_t maxNumRecords_);
                       

    /*! \brief Destructor, clears any stored data. */
    virtual ~SceneTransformationDataRecorder();

    /*! \brief Adds the scene to the transformation record
    * This method identifes all agents and objects in the scene and identifies the parent-child relationships between them.
    * \param[in] scene The scene to record.
    * \param[in] graph The graph representing the scene.
    * 
    */
    void addSceneToRecord(const ActionScene& scene, const RcsGraph* graph);

    /*! \brief Adds a body and its parent transformations to the recorder.
     *
     *  This method recursively adds the given body and its ancestors to the recording list.
     *
     * \param[in] body The body to be added.
     * \param[in,out] bodyParentMap Map of body names to their parent names.
     * \param[in] graph The RcsGraph representing the current state of the scene.
     */
    void addBodyAndParents(const RcsBody* body, std::map<std::string, std::string>& bodyParentMap, const RcsGraph* graph);

    /*! \brief Retrieves all recorded transformations.
     *
     * \return A deque containing all transformation records.
     */
    const std::deque<TransformationRecord>& getRecordedTransformations() const;


private:
     /*! \brief Event handler for post-update graph actions.
     *
     *  This method records transformations after the graph has been updated.
     *  It calculates the transformations for all tracked parent-child pairs at
     *  the current time and stores the data in a deque.
     *
     * \param[in] desired The desired state of the graph.
     * \param[in] current The current state of the graph.
     */
    void onPostUpdateGraph(RcsGraph* desired, RcsGraph* current);

    /*! \brief Records transformations at a specific time.
     *
     *  This method calculates the relative transformations for all parent-child pairs
     *  in the `bodyParentMap` and stores them as a new record in the `recordedTransformations` deque.
     *
     * \param[in] currentTime The time at which transformations are recorded.
     * \param[in] graph The graph containing the current state of the scene.
     */
    void recordTransformationsAtTime(double currentTime, const RcsGraph* graph);

    std::map<std::string, std::string> bodyParentMap; // Map of body names and their parent names
    std::deque<TransformationRecord> recordedTransformations; // Deque of recorded transformations
    size_t maxNumRecords;  // Maximum number of records to store
};

}   // namespace aff

#endif   // AFF_SCENETRANSFORMATIONDATARECORDER_H
