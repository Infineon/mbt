version=${CE_REL_VERSION}
export ASSET=${CI_PROJECT_PATH}/${CI_COMMIT_REF_NAME}
export ASSET_BUILD=${CI_PIPELINE_IID}
export ASSET_ZIP_FILE=mbt.zip
export STAGING_REPO=repo-staging/mbt
export ASSET_VERSION=$version.${CI_PIPELINE_IID}
#export STAGING_BRANCH=${CI_COMMIT_REF_NAME}

devops_scripts/job_deploy_to_staging_repo.sh --no-rebase