#! /bin/bash

echo "Removing existing build and devel directories." 
rm -rf build devel proj

echo "Building the project."
catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" 
if [ "$?" -ne 0 ]; then 
  echo "Project failed to build. Can't continue."
fi 

echo "Fixing project file." 
awk -f $(rospack find mk)/eclipse.awk build/.project > build/.project_with_env && mv build/.project_with_env build/.project
mkdir proj
mv build/.project proj/
mv build/.cproject proj/ 

echo "Fixing indexer settings." 
mkdir proj/.settings
cat <<EOF > proj/.settings/language.settings.xml
<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<project>
<configuration id="org.eclipse.cdt.core.default.config.1" name="Configuration">
<extension point="org.eclipse.cdt.core.LanguageSettingsProvider">
<provider copy-of="extension" id="org.eclipse.cdt.ui.UserLanguageSettingsProvider"/>
<provider-reference id="org.eclipse.cdt.core.ReferencedProjectsLanguageSettingsProvider" ref="shared-provider"/>
<provider-reference id="org.eclipse.cdt.core.PathEntryScannerInfoLanguageSettingsProvider" ref="shared-provider"/>
<provider copy-of="extension" id="org.eclipse.cdt.managedbuilder.core.GCCBuildCommandParser"/>
<provider-reference id="org.eclipse.cdt.managedbuilder.core.GCCBuiltinSpecsDetector" ref="shared-provider"/>
</extension>
</configuration>
</project>
EOF

echo "Changing Project Name."
cat proj/.project | perl -pe 's/<name>Project\@build<\/name>/<name>Swarmie<\/name>/' > proj/.project.rename && mv proj/.project.rename proj/.project

echo "Cleanup."
rm -rf build devel 

echo "Done."
